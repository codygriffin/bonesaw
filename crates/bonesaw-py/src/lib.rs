//! Narrow NumPy-oriented Python boundary for experiment orchestration.
//!
//! Python owns scenario construction, statistics, plotting, and reports. Rust
//! owns the model, controller state transition, and dense per-step execution.
//! The batch API writes into caller-provided NumPy arrays so long traces do not
//! create one Python object per control tick.

use std::{
    alloc::{GlobalAlloc, Layout, System},
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use bonesaw_core::{
    ActuatorEffortInput, ActuatorRealizationProfile, ActuatorRealizationState,
    ActuatorResourceModel, ActuatorResourceState, BalanceFeedbackAuthorityConfig,
    CONTACT_TRANSITION_IMPULSE_WIDTH, CONTACT_TRANSITION_WITNESS_WIDTH,
    CaptureLandingRetargetConfig, CollisionAccelerationBarrierConfig, CollisionContinuityPolicy,
    CollisionEvaluationScratch, CommandTrackingAction, CommandTrackingLimits,
    CompiledCollisionModel, CompiledFrameAtlas, CompiledWorldCollisionModel,
    CompliantContactImpulseInput, CompliantFrictionCone, CompliantStepIntegrator,
    ConservativeTerminalImpactDeltaSelection, ContactCommandLeaseConfig, ContactCommandLeaseState,
    ContactMode, ContactObservation, ContactObservationConfig, ContactObservationState,
    ContactPhaseAuthorityConfig, ContactProgramAuthorityConfig, ContactProgramAuthorityState,
    ContactSpec, ContactTransitionAccelerationIntervalInput, ContactTransitionInput,
    ContactTransitionResponseScratch, Controller, ControllerInput, ControllerOutputBuffer,
    ControllerScratch, ControllerState, CoupledContactHypothesisEnvelopeInput,
    CoupledContactImpulseInput, CoupledPositiveReferenceCompliantContactImpulseInput,
    DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH, DcmBalanceConfig, DenseSdfGrid,
    DirectionalContactTransitionInput, DistanceQuality, DistanceSample,
    DynamicTrajectoryValidationConfig, DynamicWbcConfig, DynamicsCache, ExternalFrameInputs,
    ExternalFrameSlotId, FLOATING_POINT_TASK_CAPACITY, FLOATING_TASK_DIAGNOSTIC_CAPACITY,
    FloatingCenterOfMassTask, FloatingCentroidalAngularMomentumTask, FloatingDynamicController,
    FloatingDynamicControllerInput, FloatingDynamicControllerOutput,
    FloatingDynamicControllerScratch, FloatingDynamicControllerState, FloatingDynamicWbc,
    FloatingDynamicWbcInput, FloatingDynamicWbcOutput, FloatingDynamicWbcScratch,
    FloatingFrameAngularAccelerationTask, FloatingJointAccelerationTask,
    FloatingPointAccelerationTask, FloatingRobotState, FloatingTaskPriorities, FloatingTaskWeights,
    FrameAtlasSnapshot, FrameId, FrameTarget, ModelCache,
    ModelCoupledPositiveReferenceCompliantContactImpulseInput,
    ModelCoupledPositiveReferenceContactScratch, Motion6, MotionProgram, PlanarIkOptions,
    PlanarIkScratch, PlanarPointIkTarget, PointImpulseResponseSpec,
    PositiveReferenceCompliantContactImpulseInput, Priority, ReconstructionProvenance,
    RobotObservationHistory, RobotObservationLimits, RobotObservationQueryError,
    RobotObservationQueryPolicy, RobotObservationRef, RobotObservationStamp, RobotState,
    RootPredictionErrorGrowth, SPATIAL_IMPULSE_WIDTH, SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH,
    ScalarJet, SdfOutsidePolicy, SdfSampleSource, SolveStatus, SpatialAcceleration6,
    SpatialImpulseResponseSpec, SpatialPatchTransitionInput, StepStatus, SupportContingencyConfig,
    SupportPatchSpec, SupportPhase, SupportTransitionConfig, SupportTransitionState,
    TERMINAL_IMPACT_PAIRED_COMPONENTS, TERMINAL_IMPACT_RESIDUAL_PROTOTYPE_CANDIDATES,
    TerminalImpactCandidate, TerminalImpactComponentDeltaBox, TerminalImpactConfig,
    TerminalImpactError, TerminalImpactPairedStateExemplar, TerminalImpactPairedStateTube,
    TerminalImpactResidualPrototypeProfile, TerminalImpactResidualPrototypeQuery,
    TerminalImpactScore, TerminalImpactState, TerminalImpactStateBox,
    TerminalImpactVelocityBoxState, TimingSpec, TouchdownPhaseRetimingConfig,
    TouchdownPhaseRetimingInput, Transform3, VIABILITY_EXECUTION_COMPONENTS,
    VIABILITY_FORECAST_KNOTS, Vec3, VectorJet, VelocityBounds, ViabilityConfirmationConfig,
    ViabilityConfirmationState, ViabilityExecutionMonitorConfig, ViabilityExecutionMonitorState,
    ViabilityForecastCandidate, ViabilityForecastConfig, ViabilityForecastKnot,
    ViabilityForecastState, ViabilityHybridGuardConfig, ViabilityHybridGuardState,
    ViabilityPollConfig, ViabilityPollState, ViabilityRequestConfig, ViabilityRequestState,
    WholeBodyIkOptions, WholeBodyIkScratch, WholeBodyJetOptions, WholeBodyPointIkTarget,
    WholeBodyPointJetTarget, WorldSceneStamp, WorldSceneValidity, balance_feedback_authority,
    bound_terminal_impact_paired_state_delta, capture_landing_retarget, contact_phase_authority,
    cubic_precontact_acceleration, dcm_balance_acceleration, joint_acceleration_interval,
    joint_velocity_envelope_acceleration, maximum_actuator_effort_utilization,
    minimum_joint_position_headroom, next_viability_poll, predict_viability_forecast_path,
    sample_quintic_scalar_jet, sample_quintic_vector_jet, score_terminal_impact,
    score_terminal_impact_paired_state_exemplar_delta, score_terminal_impact_state_box_upper,
    score_terminal_impact_velocity_box_upper, score_viability_forecast,
    select_conservative_terminal_impact_candidate,
    select_conservative_terminal_impact_delta_candidate, select_inexact_observation_authority,
    slew_contact_phase_authority, slew_touchdown_phase_rate, solve_coupled_contact_impulse,
    solve_coupled_positive_reference_compliant_contact_impulse,
    solve_model_coupled_positive_reference_compliant_contact_impulse, solve_planar_point_ik_into,
    solve_positive_reference_compliant_contact_impulse, solve_substepped_compliant_contact_impulse,
    solve_whole_body_ik_into, solve_whole_body_kinematic_jets_into, step_actuator_realization,
    step_actuator_resource, step_contact_command_lease, step_contact_observation,
    step_contact_program_authority_with_inexact_command, step_passive_actuator_realization,
    step_viability_confirmation, step_viability_execution_monitor, step_viability_hybrid_guard,
    step_viability_request, support_margin_phase_rate, time_warp_scalar_jet, time_warp_vector_jet,
    touchdown_phase_retiming, write_contact_transition_acceleration_interval_bounds,
    write_contact_transition_bounds, write_coupled_contact_hypothesis_velocity_envelope,
    write_directional_contact_transition_bounds, write_generalized_momentum_impulse_residuals,
    write_generalized_velocity_bounds_from_kinetic_impulse_ellipsoid,
    write_generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids,
    write_generalized_velocity_interval_from_momentum_box, write_point_impulse_velocity_response,
    write_point_impulse_velocity_response_with_delassus, write_spatial_impulse_velocity_response,
    write_spatial_patch_contact_transition_bounds, write_support_contingency_request,
    write_terminal_impact_hypothesis_envelopes,
};
use bonesaw_cuda::{
    ContactKinematicMode, ContactLockSpec, CpuExactBatchSolver, CpuExactDynamicBatchSolver,
    CpuJointEnvelopeBatchExecutor, CpuMirrorExecutor, DynamicsBatchInput, DynamicsBatchOutput,
    EmissionBatchInput, EmissionBatchOutput, ExactDynamicBatchInput, ExactDynamicBatchOutput,
    ExactDynamicSupportPatchSpec, ExactSolveBatchInput, ExactSolveBatchOutput, FkBatchInput,
    FkBatchOutput, JacobianBatchOutput, JointEnvelopeBatchInput, JointEnvelopeBatchOutput,
    POSE_COMPONENTS, PRIORITY_LEVEL_COUNT, PointAttractorSpec, PointQueryBatchOutput,
    PointQuerySpec, ROOT_TANGENT_COMPONENTS, RigidPatchBasisSpec, SPATIAL_COMPONENTS,
    derive_rigid_patch_contact_modes,
};
use bonesaw_tools::{
    UpkieCaptureReferenceConfig, UpkieCaptureReferenceState, UpkieFallSafeConfig,
    UpkieFallSafeState, UpkieLateralViabilityConfig, UpkieLateralViabilityState,
    UpkiePlanarCaptureConfig, UpkiePlanarCaptureState, UpkieWheelBalancer, UpkieWheelBalancerState,
    step_upkie_fall_safe, step_upkie_lateral_viability, write_upkie_fall_safe_contingency,
};
use nalgebra::{DMatrix, DVector, Point3, Translation3, UnitQuaternion};
use numpy::{
    PyReadonlyArray1, PyReadonlyArray2, PyReadonlyArray3, PyReadonlyArray4, PyReadwriteArray1,
    PyReadwriteArray2, PyReadwriteArray3, PyReadwriteArray4,
    ndarray::{ArrayView2, ArrayView3},
};
use pyo3::{exceptions::PyValueError, prelude::*, types::PyModule};

fn value_error(error: impl std::fmt::Display) -> pyo3::PyErr {
    PyValueError::new_err(error.to_string())
}

struct CountingAllocator;

static ALLOCATION_CALLS: AtomicU64 = AtomicU64::new(0);
static ALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);

// SAFETY: all allocation behavior is delegated unchanged to the system
// allocator; the relaxed atomics are diagnostic counters only.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed);
        // SAFETY: delegated with the exact layout received from the caller.
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        // SAFETY: delegated with the exact pointer/layout received.
        unsafe { System.dealloc(pointer, layout) }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(new_size as u64, Ordering::Relaxed);
        // SAFETY: delegated with the exact pointer/layout received.
        unsafe { System.realloc(pointer, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL_ALLOCATOR: CountingAllocator = CountingAllocator;

fn allocation_snapshot() -> (u64, u64) {
    (
        ALLOCATION_CALLS.load(Ordering::Relaxed),
        ALLOCATED_BYTES.load(Ordering::Relaxed),
    )
}

fn terminal_impact_score_from_diagnostics(values: &[f64]) -> Option<TerminalImpactScore> {
    if values.len() != 17
        || !matches!(values[0], 0.0 | 1.0)
        || values.iter().any(|value| !value.is_finite())
    {
        return None;
    }
    Some(TerminalImpactScore {
        available: values[0] != 0.0,
        time_to_impact_s: values[1],
        vertical_impact_velocity_m_s: values[2],
        vertical_specific_impact_energy_j_kg: values[3],
        terminal_tilt_rad: values[4],
        terminal_angular_rate_rad_s: values[5],
        minimum_terminal_joint_headroom_fraction: values[6],
        maximum_terminal_joint_velocity_utilization: values[7],
        impact_speed_pressure: values[8],
        tilt_pressure: values[9],
        angular_rate_pressure: values[10],
        joint_position_pressure: values[11],
        joint_velocity_pressure: values[12],
        actuator_effort_pressure: values[13],
        admission_pressure: values[14],
        maximum_terminal_harm_pressure: values[15],
        aggregate_score: values[16],
    })
}

fn write_terminal_impact_score_diagnostics(score: TerminalImpactScore, values: &mut [f64]) {
    values.copy_from_slice(&[
        f64::from(score.available),
        score.time_to_impact_s,
        score.vertical_impact_velocity_m_s,
        score.vertical_specific_impact_energy_j_kg,
        score.terminal_tilt_rad,
        score.terminal_angular_rate_rad_s,
        score.minimum_terminal_joint_headroom_fraction,
        score.maximum_terminal_joint_velocity_utilization,
        score.impact_speed_pressure,
        score.tilt_pressure,
        score.angular_rate_pressure,
        score.joint_position_pressure,
        score.joint_velocity_pressure,
        score.actuator_effort_pressure,
        score.admission_pressure,
        score.maximum_terminal_harm_pressure,
        score.aggregate_score,
    ]);
}

#[pyclass(unsendable)]
struct ActuatorResourceSession {
    models: Vec<ActuatorResourceModel>,
    states: Vec<ActuatorResourceState>,
    base_effort_limits_nm: Vec<f64>,
}

#[pyclass(unsendable)]
struct ActuatorRealizationSession {
    profiles: Vec<ActuatorRealizationProfile>,
    states: Vec<ActuatorRealizationState>,
}

#[pyclass(unsendable)]
struct ControllerSession {
    program: MotionProgram,
    controller: Controller,
    state_in: ControllerState,
    state_out: ControllerState,
    scratch: ControllerScratch,
    output: ControllerOutputBuffer,
    input: ControllerInput,
}

#[pyclass(unsendable)]
struct CollisionAvoidanceSession {
    program: MotionProgram,
    collision: CompiledCollisionModel,
    state: RobotState,
    cache: ModelCache,
    sample: DistanceSample,
    scratch: CollisionEvaluationScratch,
}

/// Fixed-capacity, allocation-stable observation authority boundary. NumPy
/// owns batch construction; Rust owns canonical ingest and reconstruction.
#[pyclass(unsendable)]
struct RobotObservationHistorySession {
    program: MotionProgram,
    history: RobotObservationHistory,
    output: FloatingRobotState,
    limits: RobotObservationLimits,
    query_policy: RobotObservationQueryPolicy,
    program_epoch: u64,
}

/// Policy- and physics-free floating-WBC query for the acceleration-level
/// collision barrier. Python owns the state corpus; Rust owns FK, dynamics,
/// closest-feature geometry, hard-row assembly, solve, and typed evidence.
#[pyclass(unsendable)]
struct FloatingCollisionBarrierSession {
    program: MotionProgram,
    controller: FloatingDynamicWbc,
    state: FloatingRobotState,
    scratch: FloatingDynamicWbcScratch,
    output: FloatingDynamicWbcOutput,
    desired_acceleration: DVector<f64>,
    acceleration_bounds: VelocityBounds,
    torque_bounds: VelocityBounds,
}

/// Policy- and physics-free floating-WBC query against an immutable dense
/// world SDF. Python authors the field and state corpus; Rust owns trilinear
/// queries, body probes, local hard-row emission, solve, and evidence.
#[pyclass(unsendable)]
struct WorldSdfBarrierSession {
    program: MotionProgram,
    controller: FloatingDynamicWbc,
    state: FloatingRobotState,
    scratch: FloatingDynamicWbcScratch,
    output: FloatingDynamicWbcOutput,
    desired_acceleration: DVector<f64>,
    acceleration_bounds: VelocityBounds,
    torque_bounds: VelocityBounds,
}

#[pyclass(unsendable)]
struct FloatingWbcSession {
    program: MotionProgram,
    controller: FloatingDynamicWbc,
    state: FloatingRobotState,
    scratch: FloatingDynamicWbcScratch,
    realization_scratch: FloatingDynamicWbcScratch,
    output: FloatingDynamicWbcOutput,
    acceleration_bounds: VelocityBounds,
    nominal_acceleration_bounds: VelocityBounds,
    torque_bounds: VelocityBounds,
    nominal_torque_bounds: VelocityBounds,
    actuator_effort_bounds: VelocityBounds,
    nominal_actuator_effort_bounds: VelocityBounds,
    mapped_actuator_effort: DVector<f64>,
    coupled_actuation_enabled: bool,
    maximum_generalized_torque: f64,
    desired_acceleration: DVector<f64>,
    protected_joint_coordinates: Vec<usize>,
    protected_joint_accelerations: Vec<f64>,
    velocity_envelope_coordinates: Vec<usize>,
    velocity_envelope_accelerations: Vec<f64>,
    joint_velocity_limits: Vec<f64>,
    tracking_cache: ModelCache,
    centroidal_dynamics: DynamicsCache,
    contact_transition_response_scratch: ContactTransitionResponseScratch,
    centroidal_map: DMatrix<f64>,
    tracking_jacobian: DMatrix<f64>,
    generalized_velocity: DVector<f64>,
    previous_center_of_mass_world: Vec3,
    point_tasks: Vec<FloatingPointAccelerationTask>,
    angular_tasks: Vec<FloatingFrameAngularAccelerationTask>,
    contacts: Vec<ContactSpec>,
    support_patches: Vec<SupportPatchSpec>,
    support_transitions: [SupportTransitionState; FLOATING_POINT_TASK_CAPACITY],
    /// Contacts that were deliberately released after an unsolved contact
    /// solve.  This is controller-owned hysteresis: keep the failed contact
    /// out of subsequent hard rows until the authored schedule releases it,
    /// rather than retrying the same expensive contact problem every tick.
    contact_release_suppressed: [bool; FLOATING_POINT_TASK_CAPACITY],
    no_contact_safe_mode: bool,
    support_transition_config: SupportTransitionConfig,
    precontact_authored_anchor_world: [Vec3; FLOATING_POINT_TASK_CAPACITY],
    precontact_anchor_world: [Vec3; FLOATING_POINT_TASK_CAPACITY],
    precontact_future_root_world: [Vec3; FLOATING_POINT_TASK_CAPACITY],
    precontact_rotation_world: [UnitQuaternion<f64>; FLOATING_POINT_TASK_CAPACITY],
    precontact_planned: [bool; FLOATING_POINT_TASK_CAPACITY],
    contact_anchor_world: [Vec3; FLOATING_POINT_TASK_CAPACITY],
    contact_patch_anchor_world: [[Vec3; 4]; FLOATING_POINT_TASK_CAPACITY],
    contact_patch_points: [Vec3; 4],
    contact_points_per_target: usize,
    minimum_contact_cop_margin_m: f64,
    maximum_contacts: usize,
    precontact_ticks: usize,
    precontact_maximum_acceleration: f64,
    material_touchdown_task: bool,
    contact_friction_coefficient: f64,
    maximum_normal_force_multiple: f64,
    maximum_acceleration: f64,
    joint_limit_braking: bool,
    root_omega: f64,
    root_angular_task_weight: f64,
    root_height_task_weight: f64,
    root_horizontal_task_weight: f64,
    root_horizontal_task_priority: Priority,
    point_omega: f64,
    joint_posture_weight: f64,
    joint_posture_priority: Priority,
    center_of_mass_task_weight: f64,
    center_of_mass_task_priority: Priority,
    center_of_mass_omega: f64,
    dcm_balance_enabled: bool,
    dcm_balance_config: DcmBalanceConfig,
    protected_joint_posture_weight: f64,
    protected_joint_posture_priority: Priority,
    joint_velocity_envelope_weight: f64,
    joint_velocity_envelope_priority: Priority,
    joint_velocity_envelope_activation_fraction: f64,
    joint_velocity_envelope_omega: f64,
    contact_phase_authority_config: ContactPhaseAuthorityConfig,
    contact_phase_authority_scale: f64,
    contact_phase_authority_maximum_delta_per_tick: f64,
    balance_feedback_authority_enabled: bool,
    balance_feedback_authority_config: BalanceFeedbackAuthorityConfig,
    capture_landing_retarget_enabled: bool,
    capture_landing_retarget_config: CaptureLandingRetargetConfig,
    capture_landing_freeze_ticks: usize,
    reference_phase_retiming_enabled: bool,
    touchdown_phase_retiming_enabled: bool,
    balance_phase_retiming_enabled: bool,
    balance_phase_hold_margin_m: f64,
    balance_phase_full_rate_margin_m: f64,
    touchdown_phase_retiming_config: TouchdownPhaseRetimingConfig,
    reference_phase: f64,
    reference_phase_rate: f64,
    previous_reference_phase_rate: f64,
    last_dcm_world: Vec3,
    last_dcm_support_margin_m: f64,
    last_dcm_observation_valid: bool,
    centroidal_angular_momentum_weight: f64,
    centroidal_angular_momentum_priority: Priority,
    centroidal_angular_momentum_omega: f64,
    supported_weight: f64,
}

/// Policy- and plant-independent model query boundary for contact-transition
/// evaluation on arbitrary URDF morphologies. Frame names are resolved and
/// query descriptors are allocated once at construction; every numerical
/// query writes into caller-owned arrays without growing Rust storage.
#[pyclass(unsendable)]
struct ContactTransitionModelSession {
    program: MotionProgram,
    robot: RobotState,
    point_specs: Vec<PointImpulseResponseSpec>,
    spatial_specs: Vec<SpatialImpulseResponseSpec>,
    scratch: ContactTransitionResponseScratch,
    coupled_impulse_scratch: Vec<f64>,
    coupled_velocity_scratch: Vec<f64>,
    coupled_delta_scratch: Vec<f64>,
    compliant_desired_velocity_delta_scratch: Vec<f64>,
    compliant_step_impulse_scratch: Vec<f64>,
    model_compliant_scratch: ModelCoupledPositiveReferenceContactScratch,
    floating_input: FloatingRobotState,
    floating_output: FloatingRobotState,
}

#[pymethods]
impl ContactTransitionModelSession {
    #[new]
    fn new(urdf_path: &str, frame_names: Vec<String>) -> PyResult<Self> {
        if frame_names.is_empty() || frame_names.len() > 64 {
            return Err(PyValueError::new_err(
                "frame_names must contain between 1 and 64 entries",
            ));
        }
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let mut point_specs = Vec::with_capacity(frame_names.len());
        let mut spatial_specs = Vec::with_capacity(frame_names.len());
        let identity_basis = [Vec3::x(), Vec3::y(), Vec3::z()];
        for name in &frame_names {
            let frame = program.model.frame_id(name).ok_or_else(|| {
                PyValueError::new_err(format!("contact-transition frame {name} is required"))
            })?;
            point_specs.push(PointImpulseResponseSpec {
                frame,
                point_world: Vec3::zeros(),
                basis_world: identity_basis,
            });
            spatial_specs.push(SpatialImpulseResponseSpec {
                frame,
                reference_point_world: Vec3::zeros(),
                basis_world: identity_basis,
            });
        }
        let robot = RobotState::zeros(&program.model);
        let scratch = ContactTransitionResponseScratch::new(&program.model);
        let contact_axes = frame_names.len() * CONTACT_TRANSITION_IMPULSE_WIDTH;
        let generalized_dof = program.model.dof + 6;
        let model_compliant_scratch =
            ModelCoupledPositiveReferenceContactScratch::new(&program.model, frame_names.len());
        let floating_input = FloatingRobotState::zeros(&program.model);
        let floating_output = FloatingRobotState::zeros(&program.model);
        Ok(Self {
            program,
            robot,
            point_specs,
            spatial_specs,
            scratch,
            coupled_impulse_scratch: vec![0.0; contact_axes],
            coupled_velocity_scratch: vec![0.0; contact_axes],
            coupled_delta_scratch: vec![0.0; generalized_dof],
            compliant_desired_velocity_delta_scratch: vec![0.0; contact_axes],
            compliant_step_impulse_scratch: vec![0.0; contact_axes],
            model_compliant_scratch,
            floating_input,
            floating_output,
        })
    }

    fn joint_dof(&self) -> usize {
        self.program.model.dof
    }

    fn generalized_dof(&self) -> usize {
        self.program.model.dof + 6
    }

    fn contact_count(&self) -> usize {
        self.point_specs.len()
    }

    fn joint_names(&self) -> Vec<&str> {
        self.program.model.coordinate_names()
    }

    fn joint_position_lower_limits(&self) -> Vec<f64> {
        let mut limits = vec![f64::NEG_INFINITY; self.program.model.dof];
        for joint in &self.program.model.joints {
            if let Some(coordinate) = joint.coordinate {
                limits[coordinate] = joint.limit.lower;
            }
        }
        limits
    }

    fn joint_position_upper_limits(&self) -> Vec<f64> {
        let mut limits = vec![f64::INFINITY; self.program.model.dof];
        for joint in &self.program.model.joints {
            if let Some(coordinate) = joint.coordinate {
                limits[coordinate] = joint.limit.upper;
            }
        }
        limits
    }

    /// Bound a contact transition for any construction-time morphology using
    /// directional slip, effective-mass, sustained-load, and componentwise
    /// continuous-acceleration witnesses. All output storage is caller-owned.
    #[allow(clippy::too_many_arguments)]
    fn bound_directional_contact_transition_velocity_jump(
        &self,
        transition_time_s: PyReadonlyArray1<'_, f64>,
        restitution_upper: f64,
        contact_witnesses: PyReadonlyArray2<'_, f64>,
        generalized_acceleration_lower: PyReadonlyArray1<'_, f64>,
        generalized_acceleration_upper: PyReadonlyArray1<'_, f64>,
        impulse_velocity_response: PyReadonlyArray3<'_, f64>,
        mut impulse_upper_out: PyReadwriteArray2<'_, f64>,
        mut delta_velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut delta_velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let witness_shape = contact_witnesses.as_array().dim();
        let response_shape = impulse_velocity_response.as_array().dim();
        let impulse_shape = impulse_upper_out.as_array().dim();
        let transition_time_s = transition_time_s.as_slice()?;
        let contact_witnesses = contact_witnesses.as_slice()?;
        let acceleration_lower = generalized_acceleration_lower.as_slice()?;
        let acceleration_upper = generalized_acceleration_upper.as_slice()?;
        let response = impulse_velocity_response.as_slice()?;
        let impulse_upper_out = impulse_upper_out.as_slice_mut()?;
        let delta_velocity_lower_out = delta_velocity_lower_out.as_slice_mut()?;
        let delta_velocity_upper_out = delta_velocity_upper_out.as_slice_mut()?;
        let generalized_dof = self.program.model.dof + 6;
        let contacts = self.point_specs.len();
        if transition_time_s.len() != 2
            || acceleration_lower.len() != generalized_dof
            || acceleration_upper.len() != generalized_dof
            || witness_shape != (contacts, DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH)
            || response_shape != (generalized_dof, contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || impulse_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || delta_velocity_lower_out.len() != generalized_dof
            || delta_velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "directional contact transition expects time[2], witnesses[{contacts},10], acceleration lower/upper[{generalized_dof}], response[{generalized_dof},{contacts},3], impulse_upper[{contacts},3], and lower/upper[{generalized_dof}]"
            )));
        }
        let input = DirectionalContactTransitionInput {
            transition_time_lower_s: transition_time_s[0],
            transition_time_upper_s: transition_time_s[1],
            restitution_upper,
            contact_witnesses,
            generalized_acceleration_lower: acceleration_lower,
            generalized_acceleration_upper: acceleration_upper,
            impulse_velocity_response: response,
        };
        write_directional_contact_transition_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid directional contact-transition bound: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_directional_contact_transition_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .expect("validated directional contact-transition bound");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "directional contact-transition bound allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Bound one or more finite contact patches as coupled resultant spatial
    /// wrenches. Force and CoP/torsional moment capacity share each patch's
    /// nonnegative normal impulse instead of forming independent point boxes.
    #[allow(clippy::too_many_arguments)]
    fn bound_spatial_patch_contact_transition_velocity_jump(
        &self,
        transition_time_s: PyReadonlyArray1<'_, f64>,
        restitution_upper: f64,
        patch_witnesses: PyReadonlyArray2<'_, f64>,
        generalized_acceleration_lower: PyReadonlyArray1<'_, f64>,
        generalized_acceleration_upper: PyReadonlyArray1<'_, f64>,
        spatial_impulse_velocity_response: PyReadonlyArray3<'_, f64>,
        mut normal_impulse_upper_out: PyReadwriteArray1<'_, f64>,
        mut delta_velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut delta_velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let witness_shape = patch_witnesses.as_array().dim();
        let response_shape = spatial_impulse_velocity_response.as_array().dim();
        let transition_time_s = transition_time_s.as_slice()?;
        let patch_witnesses = patch_witnesses.as_slice()?;
        let acceleration_lower = generalized_acceleration_lower.as_slice()?;
        let acceleration_upper = generalized_acceleration_upper.as_slice()?;
        let response = spatial_impulse_velocity_response.as_slice()?;
        let normal_impulse_upper_out = normal_impulse_upper_out.as_slice_mut()?;
        let delta_velocity_lower_out = delta_velocity_lower_out.as_slice_mut()?;
        let delta_velocity_upper_out = delta_velocity_upper_out.as_slice_mut()?;
        let generalized_dof = self.program.model.dof + 6;
        let patches = self.spatial_specs.len();
        if transition_time_s.len() != 2
            || witness_shape != (patches, SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH)
            || acceleration_lower.len() != generalized_dof
            || acceleration_upper.len() != generalized_dof
            || response_shape != (generalized_dof, patches, SPATIAL_IMPULSE_WIDTH)
            || normal_impulse_upper_out.len() != patches
            || delta_velocity_lower_out.len() != generalized_dof
            || delta_velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "spatial patch transition expects time[2], witnesses[{patches},13], acceleration lower/upper[{generalized_dof}], response[{generalized_dof},{patches},6], normal_upper[{patches}], and lower/upper[{generalized_dof}]"
            )));
        }
        let input = SpatialPatchTransitionInput {
            transition_time_lower_s: transition_time_s[0],
            transition_time_upper_s: transition_time_s[1],
            restitution_upper,
            patch_witnesses,
            generalized_acceleration_lower: acceleration_lower,
            generalized_acceleration_upper: acceleration_upper,
            spatial_impulse_velocity_response: response,
        };
        write_spatial_patch_contact_transition_bounds(
            input,
            normal_impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid spatial patch contact-transition bound: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_spatial_patch_contact_transition_bounds(
            input,
            normal_impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .expect("validated spatial patch contact-transition bound");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "spatial patch contact-transition bound allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Solve one fixed-work passive impulse over every construction-time
    /// point contact through the caller-supplied full Delassus operator.
    #[allow(clippy::too_many_arguments)]
    fn solve_coupled_contact_impulse(
        &self,
        contact_velocity: PyReadonlyArray2<'_, f64>,
        delassus: PyReadonlyArray2<'_, f64>,
        impulse_upper: PyReadonlyArray2<'_, f64>,
        friction: PyReadonlyArray1<'_, f64>,
        restitution: f64,
        diagonal_regularization_ratio: f64,
        sweeps: usize,
        mut impulse_out: PyReadwriteArray2<'_, f64>,
        mut contact_velocity_after_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let velocity_shape = contact_velocity.as_array().dim();
        let delassus_shape = delassus.as_array().dim();
        let upper_shape = impulse_upper.as_array().dim();
        let impulse_shape = impulse_out.as_array().dim();
        let after_shape = contact_velocity_after_out.as_array().dim();
        let contact_velocity = contact_velocity.as_slice()?;
        let delassus = delassus.as_slice()?;
        let impulse_upper = impulse_upper.as_slice()?;
        let friction = friction.as_slice()?;
        let impulse_out = impulse_out.as_slice_mut()?;
        let contact_velocity_after_out = contact_velocity_after_out.as_slice_mut()?;
        let contacts = self.point_specs.len();
        let axes = contacts * CONTACT_TRANSITION_IMPULSE_WIDTH;
        if velocity_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || delassus_shape != (axes, axes)
            || upper_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || friction.len() != contacts
            || impulse_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || after_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
        {
            return Err(PyValueError::new_err(format!(
                "coupled impulse expects velocity[{contacts},3], delassus[{axes},{axes}], upper[{contacts},3], friction[{contacts}], impulse[{contacts},3], and after[{contacts},3]"
            )));
        }
        let input = CoupledContactImpulseInput {
            contact_velocity,
            delassus,
            impulse_upper,
            friction,
            restitution,
            diagonal_regularization_ratio,
            sweeps,
        };
        solve_coupled_contact_impulse(input, impulse_out, contact_velocity_after_out).map_err(
            |error| PyValueError::new_err(format!("invalid coupled contact impulse: {error:?}")),
        )?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        solve_coupled_contact_impulse(input, impulse_out, contact_velocity_after_out)
            .expect("validated coupled contact impulse");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "coupled contact impulse allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Integrate one explicit compliant contact law over fixed substeps.
    #[allow(clippy::too_many_arguments)]
    fn solve_substepped_compliant_contact_impulse(
        &mut self,
        contact_gap: PyReadonlyArray1<'_, f64>,
        contact_velocity: PyReadonlyArray2<'_, f64>,
        delassus: PyReadonlyArray2<'_, f64>,
        impulse_upper: PyReadonlyArray2<'_, f64>,
        friction: PyReadonlyArray1<'_, f64>,
        normal_stiffness: PyReadonlyArray1<'_, f64>,
        normal_damping: PyReadonlyArray1<'_, f64>,
        time_step_s: f64,
        substeps: usize,
        mut impulse_out: PyReadwriteArray2<'_, f64>,
        mut contact_velocity_after_out: PyReadwriteArray2<'_, f64>,
        mut contact_gap_after_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let velocity_shape = contact_velocity.as_array().dim();
        let delassus_shape = delassus.as_array().dim();
        let upper_shape = impulse_upper.as_array().dim();
        let impulse_shape = impulse_out.as_array().dim();
        let after_shape = contact_velocity_after_out.as_array().dim();
        let contact_gap = contact_gap.as_slice()?;
        let contact_velocity = contact_velocity.as_slice()?;
        let delassus = delassus.as_slice()?;
        let impulse_upper = impulse_upper.as_slice()?;
        let friction = friction.as_slice()?;
        let normal_stiffness = normal_stiffness.as_slice()?;
        let normal_damping = normal_damping.as_slice()?;
        let impulse_out = impulse_out.as_slice_mut()?;
        let contact_velocity_after_out = contact_velocity_after_out.as_slice_mut()?;
        let contact_gap_after_out = contact_gap_after_out.as_slice_mut()?;
        let contacts = self.point_specs.len();
        let axes = contacts * CONTACT_TRANSITION_IMPULSE_WIDTH;
        if contact_gap.len() != contacts
            || velocity_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || delassus_shape != (axes, axes)
            || upper_shape != velocity_shape
            || friction.len() != contacts
            || normal_stiffness.len() != contacts
            || normal_damping.len() != contacts
            || impulse_shape != velocity_shape
            || after_shape != velocity_shape
            || contact_gap_after_out.len() != contacts
        {
            return Err(PyValueError::new_err(format!(
                "substepped compliant contact expects gap/stiffness/damping/friction[{contacts}], velocity/upper/impulse/after[{contacts},3], delassus[{axes},{axes}], and gap_after[{contacts}]"
            )));
        }
        let input = CompliantContactImpulseInput {
            contact_gap,
            contact_velocity,
            delassus,
            impulse_upper,
            friction,
            normal_stiffness,
            normal_damping,
            time_step_s,
            substeps,
        };
        solve_substepped_compliant_contact_impulse(
            input,
            &mut self.compliant_step_impulse_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid substepped compliant contact: {error:?}"))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        solve_substepped_compliant_contact_impulse(
            input,
            &mut self.compliant_step_impulse_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
        )
        .expect("validated substepped compliant contact");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "substepped compliant contact allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Integrate the positive time-constant/damping-ratio reference law with
    /// its declared impedance spline and cone geometry. Cone IDs are
    /// circular=0/pyramidal=1; integrator IDs are explicit=0, implicit=1,
    /// exponential-trapezoidal=2.
    #[allow(clippy::too_many_arguments)]
    fn solve_positive_reference_compliant_contact_impulse(
        &mut self,
        contact_gap: PyReadonlyArray1<'_, f64>,
        contact_velocity: PyReadonlyArray2<'_, f64>,
        contact_free_acceleration: PyReadonlyArray2<'_, f64>,
        delassus: PyReadonlyArray2<'_, f64>,
        impulse_upper: PyReadonlyArray2<'_, f64>,
        friction: PyReadonlyArray1<'_, f64>,
        effective_normal_mass: PyReadonlyArray1<'_, f64>,
        time_constant_s: PyReadonlyArray1<'_, f64>,
        damping_ratio: PyReadonlyArray1<'_, f64>,
        impedance_min: PyReadonlyArray1<'_, f64>,
        impedance_max: PyReadonlyArray1<'_, f64>,
        impedance_width_m: PyReadonlyArray1<'_, f64>,
        impedance_midpoint: PyReadonlyArray1<'_, f64>,
        impedance_power: PyReadonlyArray1<'_, f64>,
        minimum_time_constant_s: f64,
        time_step_s: f64,
        substeps: usize,
        friction_cone: u8,
        integrator: u8,
        mut impulse_out: PyReadwriteArray2<'_, f64>,
        mut contact_velocity_after_out: PyReadwriteArray2<'_, f64>,
        mut contact_gap_after_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let friction_cone = match friction_cone {
            0 => CompliantFrictionCone::Circular,
            1 => CompliantFrictionCone::Pyramidal,
            _ => {
                return Err(PyValueError::new_err(
                    "positive reference compliant contact cone must be circular=0 or pyramidal=1",
                ));
            }
        };
        let integrator = match integrator {
            0 => CompliantStepIntegrator::ExplicitEuler,
            1 => CompliantStepIntegrator::ImplicitEuler,
            2 => CompliantStepIntegrator::ExponentialTrapezoidal,
            _ => {
                return Err(PyValueError::new_err(
                    "positive reference compliant contact integrator must be explicit=0, implicit=1, or exponential-trapezoidal=2",
                ));
            }
        };
        let velocity_shape = contact_velocity.as_array().dim();
        let acceleration_shape = contact_free_acceleration.as_array().dim();
        let delassus_shape = delassus.as_array().dim();
        let upper_shape = impulse_upper.as_array().dim();
        let impulse_shape = impulse_out.as_array().dim();
        let after_shape = contact_velocity_after_out.as_array().dim();
        let contact_gap = contact_gap.as_slice()?;
        let contact_velocity = contact_velocity.as_slice()?;
        let contact_free_acceleration = contact_free_acceleration.as_slice()?;
        let delassus = delassus.as_slice()?;
        let impulse_upper = impulse_upper.as_slice()?;
        let friction = friction.as_slice()?;
        let effective_normal_mass = effective_normal_mass.as_slice()?;
        let time_constant_s = time_constant_s.as_slice()?;
        let damping_ratio = damping_ratio.as_slice()?;
        let impedance_min = impedance_min.as_slice()?;
        let impedance_max = impedance_max.as_slice()?;
        let impedance_width_m = impedance_width_m.as_slice()?;
        let impedance_midpoint = impedance_midpoint.as_slice()?;
        let impedance_power = impedance_power.as_slice()?;
        let impulse_out = impulse_out.as_slice_mut()?;
        let contact_velocity_after_out = contact_velocity_after_out.as_slice_mut()?;
        let contact_gap_after_out = contact_gap_after_out.as_slice_mut()?;
        let contacts = self.point_specs.len();
        let axes = contacts * CONTACT_TRANSITION_IMPULSE_WIDTH;
        let parameter_lengths = [
            contact_gap.len(),
            friction.len(),
            effective_normal_mass.len(),
            time_constant_s.len(),
            damping_ratio.len(),
            impedance_min.len(),
            impedance_max.len(),
            impedance_width_m.len(),
            impedance_midpoint.len(),
            impedance_power.len(),
            contact_gap_after_out.len(),
        ];
        if parameter_lengths.iter().any(|length| *length != contacts)
            || velocity_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || acceleration_shape != velocity_shape
            || delassus_shape != (axes, axes)
            || upper_shape != velocity_shape
            || impulse_shape != velocity_shape
            || after_shape != velocity_shape
        {
            return Err(PyValueError::new_err(format!(
                "positive reference compliant contact expects every scalar parameter[{contacts}], velocity/upper/impulse/after[{contacts},3], delassus[{axes},{axes}], and gap_after[{contacts}]"
            )));
        }
        let input = PositiveReferenceCompliantContactImpulseInput {
            contact_gap,
            contact_velocity,
            contact_free_acceleration,
            delassus,
            impulse_upper,
            friction,
            effective_normal_mass,
            time_constant_s,
            damping_ratio,
            impedance_min,
            impedance_max,
            impedance_width_m,
            impedance_midpoint,
            impedance_power,
            minimum_time_constant_s,
            time_step_s,
            substeps,
            friction_cone,
            integrator,
        };
        solve_positive_reference_compliant_contact_impulse(
            input,
            &mut self.compliant_step_impulse_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid positive reference compliant contact: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        solve_positive_reference_compliant_contact_impulse(
            input,
            &mut self.compliant_step_impulse_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
        )
        .expect("validated positive reference compliant contact");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "positive reference compliant contact allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Solve the same positive-reference law while distributing each
    /// microstep through the complete Delassus operator. Cone/integrator IDs
    /// match `solve_positive_reference_compliant_contact_impulse`.
    #[allow(clippy::too_many_arguments)]
    fn solve_coupled_positive_reference_compliant_contact_impulse(
        &mut self,
        contact_gap: PyReadonlyArray1<'_, f64>,
        contact_velocity: PyReadonlyArray2<'_, f64>,
        contact_free_acceleration: PyReadonlyArray2<'_, f64>,
        delassus: PyReadonlyArray2<'_, f64>,
        impulse_upper: PyReadonlyArray2<'_, f64>,
        friction: PyReadonlyArray1<'_, f64>,
        time_constant_s: PyReadonlyArray1<'_, f64>,
        damping_ratio: PyReadonlyArray1<'_, f64>,
        impedance_min: PyReadonlyArray1<'_, f64>,
        impedance_max: PyReadonlyArray1<'_, f64>,
        impedance_width_m: PyReadonlyArray1<'_, f64>,
        impedance_midpoint: PyReadonlyArray1<'_, f64>,
        impedance_power: PyReadonlyArray1<'_, f64>,
        minimum_time_constant_s: f64,
        time_step_s: f64,
        substeps: usize,
        projection_sweeps: usize,
        friction_cone: u8,
        integrator: u8,
        mut impulse_out: PyReadwriteArray2<'_, f64>,
        mut contact_velocity_after_out: PyReadwriteArray2<'_, f64>,
        mut contact_gap_after_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let friction_cone = match friction_cone {
            0 => CompliantFrictionCone::Circular,
            1 => CompliantFrictionCone::Pyramidal,
            _ => {
                return Err(PyValueError::new_err(
                    "coupled positive reference contact cone must be circular=0 or pyramidal=1",
                ));
            }
        };
        let integrator = match integrator {
            0 => CompliantStepIntegrator::ExplicitEuler,
            1 => CompliantStepIntegrator::ImplicitEuler,
            2 => CompliantStepIntegrator::ExponentialTrapezoidal,
            _ => {
                return Err(PyValueError::new_err(
                    "coupled positive reference contact integrator must be explicit=0, implicit=1, or exponential-trapezoidal=2",
                ));
            }
        };
        let velocity_shape = contact_velocity.as_array().dim();
        let acceleration_shape = contact_free_acceleration.as_array().dim();
        let delassus_shape = delassus.as_array().dim();
        let upper_shape = impulse_upper.as_array().dim();
        let impulse_shape = impulse_out.as_array().dim();
        let after_shape = contact_velocity_after_out.as_array().dim();
        let contact_gap = contact_gap.as_slice()?;
        let contact_velocity = contact_velocity.as_slice()?;
        let contact_free_acceleration = contact_free_acceleration.as_slice()?;
        let delassus = delassus.as_slice()?;
        let impulse_upper = impulse_upper.as_slice()?;
        let friction = friction.as_slice()?;
        let time_constant_s = time_constant_s.as_slice()?;
        let damping_ratio = damping_ratio.as_slice()?;
        let impedance_min = impedance_min.as_slice()?;
        let impedance_max = impedance_max.as_slice()?;
        let impedance_width_m = impedance_width_m.as_slice()?;
        let impedance_midpoint = impedance_midpoint.as_slice()?;
        let impedance_power = impedance_power.as_slice()?;
        let impulse_out = impulse_out.as_slice_mut()?;
        let contact_velocity_after_out = contact_velocity_after_out.as_slice_mut()?;
        let contact_gap_after_out = contact_gap_after_out.as_slice_mut()?;
        let contacts = self.point_specs.len();
        let axes = contacts * CONTACT_TRANSITION_IMPULSE_WIDTH;
        let parameter_lengths = [
            contact_gap.len(),
            friction.len(),
            time_constant_s.len(),
            damping_ratio.len(),
            impedance_min.len(),
            impedance_max.len(),
            impedance_width_m.len(),
            impedance_midpoint.len(),
            impedance_power.len(),
            contact_gap_after_out.len(),
        ];
        if parameter_lengths.iter().any(|length| *length != contacts)
            || velocity_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || acceleration_shape != velocity_shape
            || delassus_shape != (axes, axes)
            || upper_shape != velocity_shape
            || impulse_shape != velocity_shape
            || after_shape != velocity_shape
        {
            return Err(PyValueError::new_err(format!(
                "coupled positive reference contact expects every scalar parameter[{contacts}], velocity/upper/impulse/after[{contacts},3], delassus[{axes},{axes}], and gap_after[{contacts}]"
            )));
        }
        let input = CoupledPositiveReferenceCompliantContactImpulseInput {
            contact_gap,
            contact_velocity,
            contact_free_acceleration,
            delassus,
            impulse_upper,
            friction,
            time_constant_s,
            damping_ratio,
            impedance_min,
            impedance_max,
            impedance_width_m,
            impedance_midpoint,
            impedance_power,
            minimum_time_constant_s,
            time_step_s,
            substeps,
            projection_sweeps,
            friction_cone,
            integrator,
        };
        solve_coupled_positive_reference_compliant_contact_impulse(
            input,
            &mut self.compliant_desired_velocity_delta_scratch,
            &mut self.compliant_step_impulse_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid coupled positive reference compliant contact: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        solve_coupled_positive_reference_compliant_contact_impulse(
            input,
            &mut self.compliant_desired_velocity_delta_scratch,
            &mut self.compliant_step_impulse_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
        )
        .expect("validated coupled positive reference compliant contact");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "coupled positive reference compliant contact allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Advance a floating model on an independently declared state/event
    /// clock, refreshing rigid point geometry and the full Delassus operator
    /// before each event test. Integrator ids are explicit=0, implicit=1,
    /// exponential-trapezoidal=2, generalized-rk4=3,
    /// generalized-rk4-stage-force=4,
    /// generalized-implicit-stage-force=5, and
    /// generalized-explicit-activation=6. All numerical
    /// outputs are caller-owned.
    #[allow(clippy::too_many_arguments)]
    fn solve_model_coupled_positive_reference_compliant_contact_impulse(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        generalized_velocity: PyReadonlyArray1<'_, f64>,
        generalized_free_acceleration: PyReadonlyArray1<'_, f64>,
        contact_points_world: PyReadonlyArray2<'_, f64>,
        contact_bases_world: PyReadonlyArray3<'_, f64>,
        contact_surface_radius_m: PyReadonlyArray1<'_, f64>,
        initial_contact_free_acceleration: PyReadonlyArray2<'_, f64>,
        impulse_upper: PyReadonlyArray2<'_, f64>,
        friction: PyReadonlyArray1<'_, f64>,
        time_constant_s: PyReadonlyArray1<'_, f64>,
        damping_ratio: PyReadonlyArray1<'_, f64>,
        impedance_min: PyReadonlyArray1<'_, f64>,
        impedance_max: PyReadonlyArray1<'_, f64>,
        impedance_width_m: PyReadonlyArray1<'_, f64>,
        impedance_midpoint: PyReadonlyArray1<'_, f64>,
        impedance_power: PyReadonlyArray1<'_, f64>,
        plane_normal_world: PyReadonlyArray1<'_, f64>,
        plane_offset_m: f64,
        minimum_time_constant_s: f64,
        time_step_s: f64,
        state_steps: usize,
        compliance_substeps: usize,
        projection_sweeps: usize,
        friction_cone: u8,
        integrator: u8,
        mut impulse_out: PyReadwriteArray2<'_, f64>,
        mut contact_velocity_after_out: PyReadwriteArray2<'_, f64>,
        mut contact_gap_after_out: PyReadwriteArray1<'_, f64>,
        mut root_position_after_out: PyReadwriteArray1<'_, f64>,
        mut root_quaternion_wxyz_after_out: PyReadwriteArray1<'_, f64>,
        mut q_after_out: PyReadwriteArray1<'_, f64>,
        mut generalized_velocity_after_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let friction_cone = match friction_cone {
            0 => CompliantFrictionCone::Circular,
            1 => CompliantFrictionCone::Pyramidal,
            2 => CompliantFrictionCone::PyramidalEdges,
            _ => {
                return Err(PyValueError::new_err(
                    "model coupled positive reference contact cone must be circular=0, pyramidal-cartesian=1, or pyramidal-edges=2",
                ));
            }
        };
        let integrator = match integrator {
            0 => CompliantStepIntegrator::ExplicitEuler,
            1 => CompliantStepIntegrator::ImplicitEuler,
            2 => CompliantStepIntegrator::ExponentialTrapezoidal,
            // Keep ids 0/1/2 stable; model-coupled generalized RK4 gets a
            // distinct id so scalar id=2 remains exponential-trapezoidal.
            3 => CompliantStepIntegrator::GeneralizedRk4,
            4 => CompliantStepIntegrator::GeneralizedRk4StageForce,
            5 => CompliantStepIntegrator::GeneralizedImplicitStageForce,
            6 => CompliantStepIntegrator::GeneralizedExplicitActivation,
            _ => {
                return Err(PyValueError::new_err(
                    "model coupled positive reference contact integrator must be explicit=0, implicit=1, exponential-trapezoidal=2, generalized-rk4=3, generalized-rk4-stage-force=4, generalized-implicit-stage-force=5, or generalized-explicit-activation=6",
                ));
            }
        };
        let point_shape = contact_points_world.as_array().dim();
        let basis_shape = contact_bases_world.as_array().dim();
        let acceleration_shape = initial_contact_free_acceleration.as_array().dim();
        let upper_shape = impulse_upper.as_array().dim();
        let impulse_shape = impulse_out.as_array().dim();
        let velocity_after_shape = contact_velocity_after_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let generalized_velocity = generalized_velocity.as_slice()?;
        let generalized_free_acceleration = generalized_free_acceleration.as_slice()?;
        let contact_points_world = contact_points_world.as_slice()?;
        let contact_bases_world = contact_bases_world.as_slice()?;
        let contact_surface_radius_m = contact_surface_radius_m.as_slice()?;
        let initial_contact_free_acceleration = initial_contact_free_acceleration.as_slice()?;
        let impulse_upper = impulse_upper.as_slice()?;
        let friction = friction.as_slice()?;
        let time_constant_s = time_constant_s.as_slice()?;
        let damping_ratio = damping_ratio.as_slice()?;
        let impedance_min = impedance_min.as_slice()?;
        let impedance_max = impedance_max.as_slice()?;
        let impedance_width_m = impedance_width_m.as_slice()?;
        let impedance_midpoint = impedance_midpoint.as_slice()?;
        let impedance_power = impedance_power.as_slice()?;
        let plane_normal_world = plane_normal_world.as_slice()?;
        let impulse_out = impulse_out.as_slice_mut()?;
        let contact_velocity_after_out = contact_velocity_after_out.as_slice_mut()?;
        let contact_gap_after_out = contact_gap_after_out.as_slice_mut()?;
        let root_position_after_out = root_position_after_out.as_slice_mut()?;
        let root_quaternion_wxyz_after_out = root_quaternion_wxyz_after_out.as_slice_mut()?;
        let q_after_out = q_after_out.as_slice_mut()?;
        let generalized_velocity_after_out = generalized_velocity_after_out.as_slice_mut()?;
        let contacts = self.point_specs.len();
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        let parameter_lengths = [
            friction.len(),
            time_constant_s.len(),
            damping_ratio.len(),
            impedance_min.len(),
            impedance_max.len(),
            impedance_width_m.len(),
            impedance_midpoint.len(),
            impedance_power.len(),
            contact_surface_radius_m.len(),
            contact_gap_after_out.len(),
        ];
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || generalized_velocity.len() != generalized_dof
            || generalized_free_acceleration.len() != generalized_dof
            || point_shape != (contacts, 3)
            || basis_shape != (contacts, 3, 3)
            || acceleration_shape != (contacts, 3)
            || upper_shape != (contacts, 3)
            || impulse_shape != (contacts, 3)
            || velocity_after_shape != (contacts, 3)
            || parameter_lengths.iter().any(|length| *length != contacts)
            || plane_normal_world.len() != 3
            || root_position_after_out.len() != 3
            || root_quaternion_wxyz_after_out.len() != 4
            || q_after_out.len() != dof
            || generalized_velocity_after_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "model coupled contact expects root[3], quaternion[4], q[{dof}], generalized velocity/acceleration[{generalized_dof}], points[{contacts},3], bases[{contacts},3,3], point acceleration/upper/impulse/velocity[{contacts},3], scalar parameters[{contacts}], plane normal[3], gap[{contacts}], and matching state outputs"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(generalized_velocity)
            .chain(generalized_free_acceleration)
            .chain(contact_points_world)
            .chain(contact_bases_world)
            .chain(contact_surface_radius_m)
            .chain(initial_contact_free_acceleration)
            .chain(impulse_upper)
            .chain(plane_normal_world)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "model coupled contact state and geometric inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("model coupled contact quaternion is degenerate"))?;
        for (contact, spec) in self.point_specs.iter_mut().enumerate() {
            spec.point_world = Vec3::new(
                contact_points_world[contact * 3],
                contact_points_world[contact * 3 + 1],
                contact_points_world[contact * 3 + 2],
            );
            spec.basis_world = std::array::from_fn(|axis| {
                let start = contact * 9 + axis * 3;
                Vec3::new(
                    contact_bases_world[start],
                    contact_bases_world[start + 1],
                    contact_bases_world[start + 2],
                )
            });
        }
        self.floating_input.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.floating_input
            .robot
            .q
            .as_mut_slice()
            .copy_from_slice(q);
        self.floating_input
            .root_twist_world
            .0
            .as_mut_slice()
            .copy_from_slice(&generalized_velocity[..6]);
        self.floating_input
            .robot
            .v
            .as_mut_slice()
            .copy_from_slice(&generalized_velocity[6..]);
        let input = ModelCoupledPositiveReferenceCompliantContactImpulseInput {
            initial_state: &self.floating_input,
            contacts: &self.point_specs,
            contact_surface_radius_m,
            plane_normal_world: Vec3::new(
                plane_normal_world[0],
                plane_normal_world[1],
                plane_normal_world[2],
            ),
            plane_offset_m,
            generalized_free_acceleration,
            initial_contact_free_acceleration,
            impulse_upper,
            friction,
            time_constant_s,
            damping_ratio,
            impedance_min,
            impedance_max,
            impedance_width_m,
            impedance_midpoint,
            impedance_power,
            minimum_time_constant_s,
            time_step_s,
            state_steps,
            compliance_substeps,
            projection_sweeps,
            friction_cone,
            integrator,
        };
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &self.program.model,
            input,
            &mut self.model_compliant_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
            &mut self.floating_output,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid model coupled positive reference contact: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &self.program.model,
            input,
            &mut self.model_compliant_scratch,
            impulse_out,
            contact_velocity_after_out,
            contact_gap_after_out,
            &mut self.floating_output,
        )
        .expect("validated model coupled positive reference contact");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "model coupled positive reference contact allocated inside the Rust hot path",
            ));
        }
        let pose = self.floating_output.robot.control_world_from_root;
        root_position_after_out.copy_from_slice(pose.translation.vector.as_slice());
        let quaternion = pose.rotation.quaternion();
        root_quaternion_wxyz_after_out.copy_from_slice(&[
            quaternion.w,
            quaternion.i,
            quaternion.j,
            quaternion.k,
        ]);
        q_after_out.copy_from_slice(self.floating_output.robot.q.as_slice());
        generalized_velocity_after_out[..6]
            .copy_from_slice(self.floating_output.root_twist_world.0.as_slice());
        generalized_velocity_after_out[6..]
            .copy_from_slice(self.floating_output.robot.v.as_slice());
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    #[getter]
    fn terminal_impact_state_diagnostic_names(&self) -> [&'static str; 17] {
        [
            "available",
            "time_to_impact_s",
            "vertical_impact_velocity_m_s",
            "vertical_specific_impact_energy_j_kg",
            "terminal_tilt_rad",
            "terminal_angular_rate_rad_s",
            "minimum_terminal_joint_headroom_fraction",
            "maximum_terminal_joint_velocity_utilization",
            "impact_speed_pressure",
            "tilt_pressure",
            "angular_rate_pressure",
            "joint_position_pressure",
            "joint_velocity_pressure",
            "actuator_effort_pressure",
            "admission_pressure",
            "maximum_terminal_harm_pressure",
            "aggregate_score",
        ]
    }

    /// Score a bounded batch of caller-supplied terminal states. Rows may
    /// represent prediction/oracle pairs; no selection or authority is implied.
    #[allow(clippy::too_many_arguments)]
    fn score_terminal_impact_state_batch(
        &self,
        states: PyReadonlyArray2<'_, f64>,
        joint_position: PyReadonlyArray1<'_, f64>,
        joint_velocity: PyReadonlyArray2<'_, f64>,
        joint_position_lower: PyReadonlyArray1<'_, f64>,
        joint_position_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray1<'_, u8>,
        root_angular_acceleration: PyReadonlyArray2<'_, f64>,
        joint_acceleration: PyReadonlyArray2<'_, f64>,
        maximum_actuator_effort_utilization: PyReadonlyArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let state_shape = states.as_array().dim();
        let velocity_shape = joint_velocity.as_array().dim();
        let root_acceleration_shape = root_angular_acceleration.as_array().dim();
        let joint_acceleration_shape = joint_acceleration.as_array().dim();
        let diagnostic_shape = diagnostics_out.as_array().dim();
        let states = states.as_slice()?;
        let joint_position = joint_position.as_slice()?;
        let joint_velocity = joint_velocity.as_slice()?;
        let joint_position_lower = joint_position_lower.as_slice()?;
        let joint_position_upper = joint_position_upper.as_slice()?;
        let joint_velocity_limit = joint_velocity_limit.as_slice()?;
        let available = candidate_available.as_slice()?;
        let root_acceleration = root_angular_acceleration.as_slice()?;
        let joint_acceleration = joint_acceleration.as_slice()?;
        let effort = maximum_actuator_effort_utilization.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let rows = state_shape.0;
        let joints = joint_position.len();
        if rows == 0
            || rows > 64
            || state_shape != (rows, 6)
            || velocity_shape != (rows, joints)
            || joint_position_lower.len() != joints
            || joint_position_upper.len() != joints
            || joint_velocity_limit.len() != joints
            || available.len() != rows
            || available.iter().any(|value| *value > 1)
            || root_acceleration_shape != (rows, 2)
            || joint_acceleration_shape != (rows, joints)
            || effort.len() != rows
            || diagnostic_shape != (rows, 17)
        {
            return Err(PyValueError::new_err(
                "terminal state batch expects state[N,6], common joint position/limits[J], joint velocity/acceleration[N,J], availability/effort[N], root acceleration[N,2], and diagnostics[N,17] for 1<=N<=64",
            ));
        }
        let config = TerminalImpactConfig::default();
        let state_at = |row: usize| TerminalImpactState {
            root_clearance_m: states[6 * row],
            root_vertical_velocity_m_s: states[6 * row + 1],
            root_tilt_rad: [states[6 * row + 2], states[6 * row + 3]],
            root_angular_rate_rad_s: [states[6 * row + 4], states[6 * row + 5]],
            joint_position_rad: joint_position,
            joint_velocity_rad_s: &joint_velocity[row * joints..(row + 1) * joints],
            joint_position_lower_rad: joint_position_lower,
            joint_position_upper_rad: joint_position_upper,
            joint_velocity_limit_rad_s: joint_velocity_limit,
        };
        let candidate_at = |row: usize| TerminalImpactCandidate {
            available: available[row] != 0,
            root_angular_acceleration_rad_s2: [
                root_acceleration[2 * row],
                root_acceleration[2 * row + 1],
            ],
            joint_acceleration_rad_s2: &joint_acceleration[row * joints..(row + 1) * joints],
            maximum_actuator_effort_utilization: effort[row],
        };
        for row in 0..rows {
            score_terminal_impact(state_at(row), candidate_at(row), config).map_err(|error| {
                PyValueError::new_err(format!(
                    "invalid terminal-impact state row {row}: {error:?}"
                ))
            })?;
        }

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for row in 0..rows {
            let score = score_terminal_impact(state_at(row), candidate_at(row), config)
                .expect("validated terminal-impact state row");
            diagnostics[row * 17..(row + 1) * 17].copy_from_slice(&[
                f64::from(score.available),
                score.time_to_impact_s,
                score.vertical_impact_velocity_m_s,
                score.vertical_specific_impact_energy_j_kg,
                score.terminal_tilt_rad,
                score.terminal_angular_rate_rad_s,
                score.minimum_terminal_joint_headroom_fraction,
                score.maximum_terminal_joint_velocity_utilization,
                score.impact_speed_pressure,
                score.tilt_pressure,
                score.angular_rate_pressure,
                score.joint_position_pressure,
                score.joint_velocity_pressure,
                score.actuator_effort_pressure,
                score.admission_pressure,
                score.maximum_terminal_harm_pressure,
                score.aggregate_score,
            ]);
        }
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "terminal-impact state batch allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Conservatively score a bounded batch of complete terminal state boxes.
    /// Root lower/upper columns are clearance, vertical velocity, roll,
    /// pitch, roll rate, and pitch rate. Joint position and velocity are also
    /// interval-valued. This is a policy- and plant-independent consequence
    /// query; it neither chooses nor executes a command.
    #[allow(clippy::too_many_arguments)]
    fn score_terminal_impact_state_box_batch(
        &self,
        root_lower: PyReadonlyArray2<'_, f64>,
        root_upper: PyReadonlyArray2<'_, f64>,
        joint_position_lower_state: PyReadonlyArray2<'_, f64>,
        joint_position_upper_state: PyReadonlyArray2<'_, f64>,
        joint_velocity_lower: PyReadonlyArray2<'_, f64>,
        joint_velocity_upper: PyReadonlyArray2<'_, f64>,
        joint_position_lower: PyReadonlyArray1<'_, f64>,
        joint_position_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray1<'_, u8>,
        root_angular_acceleration: PyReadonlyArray2<'_, f64>,
        joint_acceleration: PyReadonlyArray2<'_, f64>,
        maximum_actuator_effort_utilization: PyReadonlyArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_lower_shape = root_lower.as_array().dim();
        let root_upper_shape = root_upper.as_array().dim();
        let position_lower_shape = joint_position_lower_state.as_array().dim();
        let position_upper_shape = joint_position_upper_state.as_array().dim();
        let velocity_lower_shape = joint_velocity_lower.as_array().dim();
        let velocity_upper_shape = joint_velocity_upper.as_array().dim();
        let root_acceleration_shape = root_angular_acceleration.as_array().dim();
        let joint_acceleration_shape = joint_acceleration.as_array().dim();
        let diagnostic_shape = diagnostics_out.as_array().dim();
        let root_lower = root_lower.as_slice()?;
        let root_upper = root_upper.as_slice()?;
        let joint_position_lower_state = joint_position_lower_state.as_slice()?;
        let joint_position_upper_state = joint_position_upper_state.as_slice()?;
        let joint_velocity_lower = joint_velocity_lower.as_slice()?;
        let joint_velocity_upper = joint_velocity_upper.as_slice()?;
        let joint_position_lower = joint_position_lower.as_slice()?;
        let joint_position_upper = joint_position_upper.as_slice()?;
        let joint_velocity_limit = joint_velocity_limit.as_slice()?;
        let available = candidate_available.as_slice()?;
        let root_acceleration = root_angular_acceleration.as_slice()?;
        let joint_acceleration = joint_acceleration.as_slice()?;
        let effort = maximum_actuator_effort_utilization.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let rows = root_lower_shape.0;
        let joints = joint_position_lower.len();
        if rows == 0
            || rows > 64
            || root_lower_shape != (rows, 6)
            || root_upper_shape != (rows, 6)
            || position_lower_shape != (rows, joints)
            || position_upper_shape != (rows, joints)
            || velocity_lower_shape != (rows, joints)
            || velocity_upper_shape != (rows, joints)
            || joint_position_upper.len() != joints
            || joint_velocity_limit.len() != joints
            || available.len() != rows
            || available.iter().any(|value| *value > 1)
            || root_acceleration_shape != (rows, 2)
            || joint_acceleration_shape != (rows, joints)
            || effort.len() != rows
            || diagnostic_shape != (rows, 17)
        {
            return Err(PyValueError::new_err(
                "terminal state box expects root lower/upper[N,6], joint position-state and velocity lower/upper[N,J], joint limits[J], availability/effort[N], root acceleration[N,2], joint acceleration[N,J], and diagnostics[N,17] for 1<=N<=64",
            ));
        }
        let config = TerminalImpactConfig::default();
        let state_at = |row: usize| TerminalImpactStateBox {
            root_clearance_lower_m: root_lower[6 * row],
            root_clearance_upper_m: root_upper[6 * row],
            root_vertical_velocity_lower_m_s: root_lower[6 * row + 1],
            root_vertical_velocity_upper_m_s: root_upper[6 * row + 1],
            root_tilt_lower_rad: [root_lower[6 * row + 2], root_lower[6 * row + 3]],
            root_tilt_upper_rad: [root_upper[6 * row + 2], root_upper[6 * row + 3]],
            root_angular_rate_lower_rad_s: [root_lower[6 * row + 4], root_lower[6 * row + 5]],
            root_angular_rate_upper_rad_s: [root_upper[6 * row + 4], root_upper[6 * row + 5]],
            joint_position_lower_state_rad: &joint_position_lower_state
                [row * joints..(row + 1) * joints],
            joint_position_upper_state_rad: &joint_position_upper_state
                [row * joints..(row + 1) * joints],
            joint_velocity_lower_rad_s: &joint_velocity_lower[row * joints..(row + 1) * joints],
            joint_velocity_upper_rad_s: &joint_velocity_upper[row * joints..(row + 1) * joints],
            joint_position_lower_rad: joint_position_lower,
            joint_position_upper_rad: joint_position_upper,
            joint_velocity_limit_rad_s: joint_velocity_limit,
        };
        let candidate_at = |row: usize| TerminalImpactCandidate {
            available: available[row] != 0,
            root_angular_acceleration_rad_s2: [
                root_acceleration[2 * row],
                root_acceleration[2 * row + 1],
            ],
            joint_acceleration_rad_s2: &joint_acceleration[row * joints..(row + 1) * joints],
            maximum_actuator_effort_utilization: effort[row],
        };
        // Validate every row before mutating caller-owned output.
        for row in 0..rows {
            score_terminal_impact_state_box_upper(state_at(row), candidate_at(row), config)
                .map_err(|error| {
                    PyValueError::new_err(format!(
                        "invalid terminal-impact state box row {row}: {error:?}"
                    ))
                })?;
        }

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for row in 0..rows {
            let score =
                score_terminal_impact_state_box_upper(state_at(row), candidate_at(row), config)
                    .expect("validated terminal-impact state box row");
            write_terminal_impact_score_diagnostics(
                score,
                &mut diagnostics[row * 17..(row + 1) * 17],
            );
        }
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "terminal-impact state box batch allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Score a fixed candidate-major table of complete terminal state boxes.
    /// This is a diagnostic table of independent consequence uppers. These
    /// outputs must not be subtracted or passed to the paired selector: an
    /// upper-minus-upper difference is not a conservative delta bound.
    #[allow(clippy::too_many_arguments)]
    fn score_terminal_impact_state_box_hypotheses(
        &self,
        root_lower: PyReadonlyArray3<'_, f64>,
        root_upper: PyReadonlyArray3<'_, f64>,
        joint_position_lower_state: PyReadonlyArray3<'_, f64>,
        joint_position_upper_state: PyReadonlyArray3<'_, f64>,
        joint_velocity_lower: PyReadonlyArray3<'_, f64>,
        joint_velocity_upper: PyReadonlyArray3<'_, f64>,
        joint_position_lower: PyReadonlyArray1<'_, f64>,
        joint_position_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray2<'_, u8>,
        root_angular_acceleration: PyReadonlyArray3<'_, f64>,
        joint_acceleration: PyReadonlyArray3<'_, f64>,
        maximum_actuator_effort_utilization: PyReadonlyArray2<'_, f64>,
        mut diagnostics_out: PyReadwriteArray3<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        const HYPOTHESES: usize = 4;
        let root_lower_shape = root_lower.as_array().dim();
        let root_upper_shape = root_upper.as_array().dim();
        let position_lower_shape = joint_position_lower_state.as_array().dim();
        let position_upper_shape = joint_position_upper_state.as_array().dim();
        let velocity_lower_shape = joint_velocity_lower.as_array().dim();
        let velocity_upper_shape = joint_velocity_upper.as_array().dim();
        let available_shape = candidate_available.as_array().dim();
        let root_acceleration_shape = root_angular_acceleration.as_array().dim();
        let joint_acceleration_shape = joint_acceleration.as_array().dim();
        let effort_shape = maximum_actuator_effort_utilization.as_array().dim();
        let diagnostic_shape = diagnostics_out.as_array().dim();
        let root_lower = root_lower.as_slice()?;
        let root_upper = root_upper.as_slice()?;
        let joint_position_lower_state = joint_position_lower_state.as_slice()?;
        let joint_position_upper_state = joint_position_upper_state.as_slice()?;
        let joint_velocity_lower = joint_velocity_lower.as_slice()?;
        let joint_velocity_upper = joint_velocity_upper.as_slice()?;
        let joint_position_lower = joint_position_lower.as_slice()?;
        let joint_position_upper = joint_position_upper.as_slice()?;
        let joint_velocity_limit = joint_velocity_limit.as_slice()?;
        let available = candidate_available.as_slice()?;
        let root_acceleration = root_angular_acceleration.as_slice()?;
        let joint_acceleration = joint_acceleration.as_slice()?;
        let effort = maximum_actuator_effort_utilization.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let joints = joint_position_lower.len();
        if root_lower_shape != (CANDIDATES, HYPOTHESES, 6)
            || root_upper_shape != (CANDIDATES, HYPOTHESES, 6)
            || position_lower_shape != (CANDIDATES, HYPOTHESES, joints)
            || position_upper_shape != (CANDIDATES, HYPOTHESES, joints)
            || velocity_lower_shape != (CANDIDATES, HYPOTHESES, joints)
            || velocity_upper_shape != (CANDIDATES, HYPOTHESES, joints)
            || joint_position_upper.len() != joints
            || joint_velocity_limit.len() != joints
            || available_shape != (CANDIDATES, HYPOTHESES)
            || available.iter().any(|value| *value > 1)
            || root_acceleration_shape != (CANDIDATES, HYPOTHESES, 2)
            || joint_acceleration_shape != (CANDIDATES, HYPOTHESES, joints)
            || effort_shape != (CANDIDATES, HYPOTHESES)
            || diagnostic_shape != (CANDIDATES, HYPOTHESES, 17)
        {
            return Err(PyValueError::new_err(
                "terminal state-box hypotheses expect root lower/upper[3,4,6], position/velocity lower/upper[3,4,J], joint limits[J], availability/effort[3,4], root acceleration[3,4,2], joint acceleration[3,4,J], and diagnostics[3,4,17]",
            ));
        }
        let config = TerminalImpactConfig::default();
        let state_at = |candidate: usize, hypothesis: usize| {
            let root = (candidate * HYPOTHESES + hypothesis) * 6;
            let row = candidate * HYPOTHESES + hypothesis;
            let joints_start = row * joints;
            TerminalImpactStateBox {
                root_clearance_lower_m: root_lower[root],
                root_clearance_upper_m: root_upper[root],
                root_vertical_velocity_lower_m_s: root_lower[root + 1],
                root_vertical_velocity_upper_m_s: root_upper[root + 1],
                root_tilt_lower_rad: [root_lower[root + 2], root_lower[root + 3]],
                root_tilt_upper_rad: [root_upper[root + 2], root_upper[root + 3]],
                root_angular_rate_lower_rad_s: [root_lower[root + 4], root_lower[root + 5]],
                root_angular_rate_upper_rad_s: [root_upper[root + 4], root_upper[root + 5]],
                joint_position_lower_state_rad: &joint_position_lower_state
                    [joints_start..joints_start + joints],
                joint_position_upper_state_rad: &joint_position_upper_state
                    [joints_start..joints_start + joints],
                joint_velocity_lower_rad_s: &joint_velocity_lower
                    [joints_start..joints_start + joints],
                joint_velocity_upper_rad_s: &joint_velocity_upper
                    [joints_start..joints_start + joints],
                joint_position_lower_rad: joint_position_lower,
                joint_position_upper_rad: joint_position_upper,
                joint_velocity_limit_rad_s: joint_velocity_limit,
            }
        };
        let candidate_at = |candidate: usize, hypothesis: usize| {
            let row = candidate * HYPOTHESES + hypothesis;
            let joints_start = row * joints;
            TerminalImpactCandidate {
                available: available[row] != 0,
                root_angular_acceleration_rad_s2: [
                    root_acceleration[row * 2],
                    root_acceleration[row * 2 + 1],
                ],
                joint_acceleration_rad_s2: &joint_acceleration[joints_start..joints_start + joints],
                maximum_actuator_effort_utilization: effort[row],
            }
        };
        for candidate in 0..CANDIDATES {
            for hypothesis in 0..HYPOTHESES {
                score_terminal_impact_state_box_upper(
                    state_at(candidate, hypothesis),
                    candidate_at(candidate, hypothesis),
                    config,
                )
                .map_err(|error| {
                    PyValueError::new_err(format!(
                        "invalid terminal state-box hypothesis [{candidate},{hypothesis}]: {error:?}"
                    ))
                })?;
            }
        }

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for candidate in 0..CANDIDATES {
            for hypothesis in 0..HYPOTHESES {
                let score = score_terminal_impact_state_box_upper(
                    state_at(candidate, hypothesis),
                    candidate_at(candidate, hypothesis),
                    config,
                )
                .expect("validated terminal state-box hypothesis");
                let output = (candidate * HYPOTHESES + hypothesis) * 17;
                write_terminal_impact_score_diagnostics(
                    score,
                    &mut diagnostics[output..output + 17],
                );
            }
        }
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "terminal state-box hypothesis batch allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Bound and conservatively select exactly three candidates across four
    /// shared terminal-state hypotheses. Baseline boxes are indexed only by
    /// hypothesis; every candidate is `that same baseline + its delta tube`.
    /// Hypothesis diagnostics and envelopes use `[lower(6), upper(6),
    /// aggregate_lower, aggregate_upper]`. This query emits no command.
    #[allow(clippy::too_many_arguments)]
    fn bound_terminal_impact_paired_state_tubes(
        &self,
        baseline_root_lower: PyReadonlyArray2<'_, f64>,
        baseline_root_upper: PyReadonlyArray2<'_, f64>,
        candidate_root_delta_lower: PyReadonlyArray3<'_, f64>,
        candidate_root_delta_upper: PyReadonlyArray3<'_, f64>,
        baseline_joint_position_lower: PyReadonlyArray2<'_, f64>,
        baseline_joint_position_upper: PyReadonlyArray2<'_, f64>,
        candidate_joint_position_delta_lower: PyReadonlyArray3<'_, f64>,
        candidate_joint_position_delta_upper: PyReadonlyArray3<'_, f64>,
        baseline_joint_velocity_lower: PyReadonlyArray2<'_, f64>,
        baseline_joint_velocity_upper: PyReadonlyArray2<'_, f64>,
        candidate_joint_velocity_delta_lower: PyReadonlyArray3<'_, f64>,
        candidate_joint_velocity_delta_upper: PyReadonlyArray3<'_, f64>,
        joint_position_limit_lower: PyReadonlyArray1<'_, f64>,
        joint_position_limit_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray2<'_, u8>,
        baseline_effort_utilization: PyReadonlyArray1<'_, f64>,
        candidate_effort_utilization: PyReadonlyArray2<'_, f64>,
        baseline_index: usize,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut hypothesis_diagnostics_out: PyReadwriteArray3<'_, f64>,
        mut envelope_diagnostics_out: PyReadwriteArray2<'_, f64>,
        mut selection_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        const HYPOTHESES: usize = 4;
        const COMPONENTS: usize = TERMINAL_IMPACT_PAIRED_COMPONENTS;
        const DIAGNOSTICS: usize = 2 * COMPONENTS + 2;
        let baseline_root_lower_shape = baseline_root_lower.as_array().dim();
        let baseline_root_upper_shape = baseline_root_upper.as_array().dim();
        let candidate_root_delta_lower_shape = candidate_root_delta_lower.as_array().dim();
        let candidate_root_delta_upper_shape = candidate_root_delta_upper.as_array().dim();
        let baseline_position_lower_shape = baseline_joint_position_lower.as_array().dim();
        let baseline_position_upper_shape = baseline_joint_position_upper.as_array().dim();
        let candidate_position_delta_lower_shape =
            candidate_joint_position_delta_lower.as_array().dim();
        let candidate_position_delta_upper_shape =
            candidate_joint_position_delta_upper.as_array().dim();
        let baseline_velocity_lower_shape = baseline_joint_velocity_lower.as_array().dim();
        let baseline_velocity_upper_shape = baseline_joint_velocity_upper.as_array().dim();
        let candidate_velocity_delta_lower_shape =
            candidate_joint_velocity_delta_lower.as_array().dim();
        let candidate_velocity_delta_upper_shape =
            candidate_joint_velocity_delta_upper.as_array().dim();
        let available_shape = candidate_available.as_array().dim();
        let candidate_effort_shape = candidate_effort_utilization.as_array().dim();
        let hypothesis_output_shape = hypothesis_diagnostics_out.as_array().dim();
        let envelope_output_shape = envelope_diagnostics_out.as_array().dim();
        let baseline_root_lower = baseline_root_lower.as_slice()?;
        let baseline_root_upper = baseline_root_upper.as_slice()?;
        let candidate_root_delta_lower = candidate_root_delta_lower.as_slice()?;
        let candidate_root_delta_upper = candidate_root_delta_upper.as_slice()?;
        let baseline_position_lower = baseline_joint_position_lower.as_slice()?;
        let baseline_position_upper = baseline_joint_position_upper.as_slice()?;
        let candidate_position_delta_lower = candidate_joint_position_delta_lower.as_slice()?;
        let candidate_position_delta_upper = candidate_joint_position_delta_upper.as_slice()?;
        let baseline_velocity_lower = baseline_joint_velocity_lower.as_slice()?;
        let baseline_velocity_upper = baseline_joint_velocity_upper.as_slice()?;
        let candidate_velocity_delta_lower = candidate_joint_velocity_delta_lower.as_slice()?;
        let candidate_velocity_delta_upper = candidate_joint_velocity_delta_upper.as_slice()?;
        let position_limit_lower = joint_position_limit_lower.as_slice()?;
        let position_limit_upper = joint_position_limit_upper.as_slice()?;
        let velocity_limit = joint_velocity_limit.as_slice()?;
        let available = candidate_available.as_slice()?;
        let baseline_effort = baseline_effort_utilization.as_slice()?;
        let candidate_effort = candidate_effort_utilization.as_slice()?;
        let hypothesis_output = hypothesis_diagnostics_out.as_slice_mut()?;
        let envelope_output = envelope_diagnostics_out.as_slice_mut()?;
        let selection_output = selection_out.as_slice_mut()?;
        let joints = position_limit_lower.len();
        if baseline_root_lower_shape != (HYPOTHESES, 4)
            || baseline_root_upper_shape != (HYPOTHESES, 4)
            || candidate_root_delta_lower_shape != (CANDIDATES, HYPOTHESES, 4)
            || candidate_root_delta_upper_shape != (CANDIDATES, HYPOTHESES, 4)
            || baseline_position_lower_shape != (HYPOTHESES, joints)
            || baseline_position_upper_shape != (HYPOTHESES, joints)
            || candidate_position_delta_lower_shape != (CANDIDATES, HYPOTHESES, joints)
            || candidate_position_delta_upper_shape != (CANDIDATES, HYPOTHESES, joints)
            || baseline_velocity_lower_shape != (HYPOTHESES, joints)
            || baseline_velocity_upper_shape != (HYPOTHESES, joints)
            || candidate_velocity_delta_lower_shape != (CANDIDATES, HYPOTHESES, joints)
            || candidate_velocity_delta_upper_shape != (CANDIDATES, HYPOTHESES, joints)
            || position_limit_upper.len() != joints
            || velocity_limit.len() != joints
            || available_shape != (CANDIDATES, HYPOTHESES)
            || available.iter().any(|value| *value > 1)
            || baseline_effort.len() != HYPOTHESES
            || candidate_effort_shape != (CANDIDATES, HYPOTHESES)
            || baseline_index >= CANDIDATES
            || hypothesis_output_shape != (CANDIDATES, HYPOTHESES, DIAGNOSTICS)
            || envelope_output_shape != (CANDIDATES, DIAGNOSTICS)
            || selection_output.len() != 6
        {
            return Err(PyValueError::new_err(
                "paired terminal state tubes expect baseline root[4,4], candidate root deltas[3,4,4], baseline joint state[4,J], candidate joint deltas[3,4,J], limits[J], availability/candidate effort[3,4], baseline effort[4], hypothesis output[3,4,14], envelope output[3,14], and selection[6]",
            ));
        }
        let config = TerminalImpactConfig::default();
        let build = || -> Result<
            (
                [TerminalImpactComponentDeltaBox; CANDIDATES * HYPOTHESES],
                [TerminalImpactComponentDeltaBox; CANDIDATES],
                ConservativeTerminalImpactDeltaSelection,
            ),
            TerminalImpactError,
        > {
            let zero = TerminalImpactComponentDeltaBox {
                available: false,
                component_lower: [0.0; COMPONENTS],
                component_upper: [0.0; COMPONENTS],
                aggregate_lower: 0.0,
                aggregate_upper: 0.0,
            };
            let mut hypotheses = [zero; CANDIDATES * HYPOTHESES];
            for candidate in 0..CANDIDATES {
                for hypothesis in 0..HYPOTHESES {
                    let row = candidate * HYPOTHESES + hypothesis;
                    let root = hypothesis * 4;
                    let candidate_root = row * 4;
                    let baseline_joint = hypothesis * joints;
                    let candidate_joint = row * joints;
                    hypotheses[row] = bound_terminal_impact_paired_state_delta(
                        TerminalImpactPairedStateTube {
                            available: available[row] != 0,
                            baseline_tilt_lower_rad: [
                                baseline_root_lower[root],
                                baseline_root_lower[root + 1],
                            ],
                            baseline_tilt_upper_rad: [
                                baseline_root_upper[root],
                                baseline_root_upper[root + 1],
                            ],
                            candidate_tilt_delta_lower_rad: [
                                candidate_root_delta_lower[candidate_root],
                                candidate_root_delta_lower[candidate_root + 1],
                            ],
                            candidate_tilt_delta_upper_rad: [
                                candidate_root_delta_upper[candidate_root],
                                candidate_root_delta_upper[candidate_root + 1],
                            ],
                            baseline_angular_rate_lower_rad_s: [
                                baseline_root_lower[root + 2],
                                baseline_root_lower[root + 3],
                            ],
                            baseline_angular_rate_upper_rad_s: [
                                baseline_root_upper[root + 2],
                                baseline_root_upper[root + 3],
                            ],
                            candidate_angular_rate_delta_lower_rad_s: [
                                candidate_root_delta_lower[candidate_root + 2],
                                candidate_root_delta_lower[candidate_root + 3],
                            ],
                            candidate_angular_rate_delta_upper_rad_s: [
                                candidate_root_delta_upper[candidate_root + 2],
                                candidate_root_delta_upper[candidate_root + 3],
                            ],
                            baseline_joint_position_lower_rad: &baseline_position_lower
                                [baseline_joint..baseline_joint + joints],
                            baseline_joint_position_upper_rad: &baseline_position_upper
                                [baseline_joint..baseline_joint + joints],
                            candidate_joint_position_delta_lower_rad: &candidate_position_delta_lower
                                [candidate_joint..candidate_joint + joints],
                            candidate_joint_position_delta_upper_rad: &candidate_position_delta_upper
                                [candidate_joint..candidate_joint + joints],
                            baseline_joint_velocity_lower_rad_s: &baseline_velocity_lower
                                [baseline_joint..baseline_joint + joints],
                            baseline_joint_velocity_upper_rad_s: &baseline_velocity_upper
                                [baseline_joint..baseline_joint + joints],
                            candidate_joint_velocity_delta_lower_rad_s: &candidate_velocity_delta_lower
                                [candidate_joint..candidate_joint + joints],
                            candidate_joint_velocity_delta_upper_rad_s: &candidate_velocity_delta_upper
                                [candidate_joint..candidate_joint + joints],
                            joint_position_limit_lower_rad: position_limit_lower,
                            joint_position_limit_upper_rad: position_limit_upper,
                            joint_velocity_limit_rad_s: velocity_limit,
                            baseline_actuator_effort_utilization: baseline_effort[hypothesis],
                            candidate_actuator_effort_utilization: candidate_effort[row],
                        },
                        config,
                    )?;
                }
            }
            let mut envelopes = [zero; CANDIDATES];
            for candidate in 0..CANDIDATES {
                let mut envelope = hypotheses[candidate * HYPOTHESES];
                for hypothesis in 1..HYPOTHESES {
                    let value = hypotheses[candidate * HYPOTHESES + hypothesis];
                    envelope.available &= value.available;
                    for component in 0..COMPONENTS {
                        envelope.component_lower[component] = envelope.component_lower[component]
                            .min(value.component_lower[component]);
                        envelope.component_upper[component] = envelope.component_upper[component]
                            .max(value.component_upper[component]);
                    }
                    envelope.aggregate_lower = envelope.aggregate_lower.min(value.aggregate_lower);
                    envelope.aggregate_upper = envelope.aggregate_upper.max(value.aggregate_upper);
                }
                envelopes[candidate] = envelope;
            }
            let selection = select_conservative_terminal_impact_delta_candidate(
                &envelopes,
                baseline_index,
                maximum_component_regression,
                minimum_component_improvement,
            )?;
            Ok((hypotheses, envelopes, selection))
        };

        build().map_err(|error| {
            PyValueError::new_err(format!("invalid paired terminal state tube: {error:?}"))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let (hypotheses, envelopes, selection) = build().map_err(|error| {
            PyValueError::new_err(format!("invalid paired terminal state tube: {error:?}"))
        })?;
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "paired terminal state tube allocated inside the Rust hot path",
            ));
        }
        let write_box = |value: TerminalImpactComponentDeltaBox, output: &mut [f64]| {
            output[..COMPONENTS].copy_from_slice(&value.component_lower);
            output[COMPONENTS..2 * COMPONENTS].copy_from_slice(&value.component_upper);
            output[2 * COMPONENTS] = value.aggregate_lower;
            output[2 * COMPONENTS + 1] = value.aggregate_upper;
        };
        for (value, output) in hypotheses
            .into_iter()
            .zip(hypothesis_output.chunks_exact_mut(DIAGNOSTICS))
        {
            write_box(value, output);
        }
        for (value, output) in envelopes
            .into_iter()
            .zip(envelope_output.chunks_exact_mut(DIAGNOSTICS))
        {
            write_box(value, output);
        }
        selection_output.copy_from_slice(&[
            selection.selected_index as f64,
            selection.baseline_index as f64,
            selection.maximum_component_delta_upper,
            selection.maximum_guaranteed_component_improvement,
            selection.aggregate_delta_lower,
            selection.aggregate_delta_upper,
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Score exactly three candidates across 1..=16 correlated complete-state
    /// exemplars. Each hypothesis index pairs its baseline and candidate before
    /// scoring; clearance and vertical velocity remain candidate-dependent.
    /// The query only emits consequence boxes and a conservative selection.
    #[allow(clippy::too_many_arguments)]
    fn score_terminal_impact_paired_state_exemplars(
        &self,
        terminal_root_state: PyReadonlyArray3<'_, f64>,
        terminal_joint_position: PyReadonlyArray3<'_, f64>,
        terminal_joint_velocity: PyReadonlyArray3<'_, f64>,
        joint_position_limit_lower: PyReadonlyArray1<'_, f64>,
        joint_position_limit_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        support_free_joint_acceleration: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray2<'_, u8>,
        candidate_effort_utilization: PyReadonlyArray2<'_, f64>,
        baseline_index: usize,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut hypothesis_diagnostics_out: PyReadwriteArray3<'_, f64>,
        mut envelope_diagnostics_out: PyReadwriteArray2<'_, f64>,
        mut selection_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        const MAX_HYPOTHESES: usize = 16;
        const COMPONENTS: usize = TERMINAL_IMPACT_PAIRED_COMPONENTS;
        const DIAGNOSTICS: usize = 2 * COMPONENTS + 2;
        let root_shape = terminal_root_state.as_array().dim();
        let position_shape = terminal_joint_position.as_array().dim();
        let velocity_shape = terminal_joint_velocity.as_array().dim();
        let available_shape = candidate_available.as_array().dim();
        let effort_shape = candidate_effort_utilization.as_array().dim();
        let hypothesis_output_shape = hypothesis_diagnostics_out.as_array().dim();
        let envelope_output_shape = envelope_diagnostics_out.as_array().dim();
        let hypotheses_per_candidate = root_shape.1;
        let root = terminal_root_state.as_slice()?;
        let position = terminal_joint_position.as_slice()?;
        let velocity = terminal_joint_velocity.as_slice()?;
        let position_limit_lower = joint_position_limit_lower.as_slice()?;
        let position_limit_upper = joint_position_limit_upper.as_slice()?;
        let velocity_limit = joint_velocity_limit.as_slice()?;
        let zero_acceleration = support_free_joint_acceleration.as_slice()?;
        let available = candidate_available.as_slice()?;
        let effort = candidate_effort_utilization.as_slice()?;
        let hypothesis_output = hypothesis_diagnostics_out.as_slice_mut()?;
        let envelope_output = envelope_diagnostics_out.as_slice_mut()?;
        let selection_output = selection_out.as_slice_mut()?;
        let joints = position_limit_lower.len();
        if hypotheses_per_candidate == 0
            || hypotheses_per_candidate > MAX_HYPOTHESES
            || root_shape != (CANDIDATES, hypotheses_per_candidate, 6)
            || position_shape != (CANDIDATES, hypotheses_per_candidate, joints)
            || velocity_shape != (CANDIDATES, hypotheses_per_candidate, joints)
            || position_limit_upper.len() != joints
            || velocity_limit.len() != joints
            || zero_acceleration.len() != joints
            || available_shape != (CANDIDATES, hypotheses_per_candidate)
            || available.iter().any(|value| *value > 1)
            || effort_shape != (CANDIDATES, hypotheses_per_candidate)
            || baseline_index >= CANDIDATES
            || hypothesis_output_shape != (CANDIDATES, hypotheses_per_candidate, DIAGNOSTICS)
            || envelope_output_shape != (CANDIDATES, DIAGNOSTICS)
            || selection_output.len() != 6
        {
            return Err(PyValueError::new_err(
                "paired terminal-state exemplars expect root[3,H,6], joint position/velocity[3,H,J], limits and zero acceleration[J], availability/effort[3,H], hypothesis output[3,H,14], envelope output[3,14], selection[6], and 1<=H<=16",
            ));
        }
        let config = TerminalImpactConfig::default();
        let build = || -> Result<
            (
                [TerminalImpactComponentDeltaBox; CANDIDATES * MAX_HYPOTHESES],
                [TerminalImpactComponentDeltaBox; CANDIDATES],
                ConservativeTerminalImpactDeltaSelection,
            ),
            TerminalImpactError,
        > {
            let zero = TerminalImpactComponentDeltaBox {
                available: false,
                component_lower: [0.0; COMPONENTS],
                component_upper: [0.0; COMPONENTS],
                aggregate_lower: 0.0,
                aggregate_upper: 0.0,
            };
            let mut hypotheses = [zero; CANDIDATES * MAX_HYPOTHESES];
            let state_at = |candidate: usize, hypothesis: usize| {
                let row = candidate * hypotheses_per_candidate + hypothesis;
                let root_at = row * 6;
                let joint_at = row * joints;
                TerminalImpactState {
                    root_clearance_m: root[root_at],
                    root_vertical_velocity_m_s: root[root_at + 1],
                    root_tilt_rad: [root[root_at + 2], root[root_at + 3]],
                    root_angular_rate_rad_s: [root[root_at + 4], root[root_at + 5]],
                    joint_position_rad: &position[joint_at..joint_at + joints],
                    joint_velocity_rad_s: &velocity[joint_at..joint_at + joints],
                    joint_position_lower_rad: position_limit_lower,
                    joint_position_upper_rad: position_limit_upper,
                    joint_velocity_limit_rad_s: velocity_limit,
                }
            };
            for candidate in 0..CANDIDATES {
                for hypothesis in 0..hypotheses_per_candidate {
                    let row = candidate * hypotheses_per_candidate + hypothesis;
                    let baseline_row = baseline_index * hypotheses_per_candidate + hypothesis;
                    hypotheses[row] = score_terminal_impact_paired_state_exemplar_delta(
                        TerminalImpactPairedStateExemplar {
                            available: available[row] != 0,
                            baseline_state: state_at(baseline_index, hypothesis),
                            candidate_state: state_at(candidate, hypothesis),
                            support_free_joint_acceleration_rad_s2: zero_acceleration,
                            baseline_actuator_effort_utilization: effort[baseline_row],
                            candidate_actuator_effort_utilization: effort[row],
                        },
                        config,
                    )?;
                }
            }
            let mut envelopes = [zero; CANDIDATES];
            for candidate in 0..CANDIDATES {
                let start = candidate * hypotheses_per_candidate;
                let mut envelope = hypotheses[start];
                for value in &hypotheses[start + 1..start + hypotheses_per_candidate] {
                    envelope.available &= value.available;
                    for component in 0..COMPONENTS {
                        envelope.component_lower[component] =
                            envelope.component_lower[component].min(value.component_lower[component]);
                        envelope.component_upper[component] =
                            envelope.component_upper[component].max(value.component_upper[component]);
                    }
                    envelope.aggregate_lower = envelope.aggregate_lower.min(value.aggregate_lower);
                    envelope.aggregate_upper = envelope.aggregate_upper.max(value.aggregate_upper);
                }
                envelopes[candidate] = envelope;
            }
            let selection = select_conservative_terminal_impact_delta_candidate(
                &envelopes,
                baseline_index,
                maximum_component_regression,
                minimum_component_improvement,
            )?;
            Ok((hypotheses, envelopes, selection))
        };

        build().map_err(|error| {
            PyValueError::new_err(format!(
                "invalid paired terminal-state exemplars: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let (hypotheses, envelopes, selection) = build().map_err(|error| {
            PyValueError::new_err(format!(
                "invalid paired terminal-state exemplars: {error:?}"
            ))
        })?;
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "paired terminal-state exemplar query allocated inside the Rust hot path",
            ));
        }
        let write_box = |value: TerminalImpactComponentDeltaBox, output: &mut [f64]| {
            output[..COMPONENTS].copy_from_slice(&value.component_lower);
            output[COMPONENTS..2 * COMPONENTS].copy_from_slice(&value.component_upper);
            output[2 * COMPONENTS] = value.aggregate_lower;
            output[2 * COMPONENTS + 1] = value.aggregate_upper;
        };
        for candidate in 0..CANDIDATES {
            let start = candidate * hypotheses_per_candidate;
            for (value, output) in hypotheses[start..start + hypotheses_per_candidate]
                .iter()
                .copied()
                .zip(
                    hypothesis_output[candidate * hypotheses_per_candidate * DIAGNOSTICS
                        ..(candidate + 1) * hypotheses_per_candidate * DIAGNOSTICS]
                        .chunks_exact_mut(DIAGNOSTICS),
                )
            {
                write_box(value, output);
            }
        }
        for (value, output) in envelopes
            .into_iter()
            .zip(envelope_output.chunks_exact_mut(DIAGNOSTICS))
        {
            write_box(value, output);
        }
        selection_output.copy_from_slice(&[
            selection.selected_index as f64,
            selection.baseline_index as f64,
            selection.maximum_component_delta_upper,
            selection.maximum_guaranteed_component_improvement,
            selection.aggregate_delta_lower,
            selection.aggregate_delta_upper,
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Apply one immutable causal nearest-residual profile to a batch of
    /// predicted paired terminal deltas. Static profile storage is validated
    /// once; each row then executes a bounded allocation-free Rust query.
    #[allow(clippy::too_many_arguments)]
    fn score_terminal_impact_residual_prototype_profile(
        &self,
        query_features: PyReadonlyArray2<'_, f64>,
        query_groups: PyReadonlyArray1<'_, u8>,
        predicted_component_delta: PyReadonlyArray3<'_, f64>,
        predicted_aggregate_delta: PyReadonlyArray2<'_, f64>,
        candidate_available: PyReadonlyArray2<'_, u8>,
        feature_inverse_scale: PyReadonlyArray1<'_, f64>,
        prototype_features: PyReadonlyArray2<'_, f64>,
        prototype_groups: PyReadonlyArray1<'_, u8>,
        prototype_component_residuals: PyReadonlyArray3<'_, f64>,
        prototype_aggregate_residuals: PyReadonlyArray2<'_, f64>,
        group_component_lower_extension: PyReadonlyArray3<'_, f64>,
        group_component_upper_extension: PyReadonlyArray3<'_, f64>,
        group_aggregate_lower_extension: PyReadonlyArray2<'_, f64>,
        group_aggregate_upper_extension: PyReadonlyArray2<'_, f64>,
        maximum_distance_squared_by_group: PyReadonlyArray1<'_, f64>,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut envelope_diagnostics_out: PyReadwriteArray3<'_, f64>,
        mut selection_out: PyReadwriteArray2<'_, f64>,
        mut nearest_prototype_index_out: PyReadwriteArray1<'_, i64>,
        mut nearest_distance_squared_out: PyReadwriteArray1<'_, f64>,
        mut profile_supported_out: PyReadwriteArray1<'_, u8>,
        mut timing_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        const CANDIDATES: usize = TERMINAL_IMPACT_RESIDUAL_PROTOTYPE_CANDIDATES;
        const COMPONENTS: usize = TERMINAL_IMPACT_PAIRED_COMPONENTS;
        const DIAGNOSTICS: usize = 2 * COMPONENTS + 2;
        let query_shape = query_features.as_array().dim();
        let predicted_component_shape = predicted_component_delta.as_array().dim();
        let predicted_aggregate_shape = predicted_aggregate_delta.as_array().dim();
        let available_shape = candidate_available.as_array().dim();
        let prototype_shape = prototype_features.as_array().dim();
        let prototype_component_shape = prototype_component_residuals.as_array().dim();
        let prototype_aggregate_shape = prototype_aggregate_residuals.as_array().dim();
        let group_component_lower_shape = group_component_lower_extension.as_array().dim();
        let group_component_upper_shape = group_component_upper_extension.as_array().dim();
        let group_aggregate_lower_shape = group_aggregate_lower_extension.as_array().dim();
        let group_aggregate_upper_shape = group_aggregate_upper_extension.as_array().dim();
        let envelope_shape = envelope_diagnostics_out.as_array().dim();
        let selection_shape = selection_out.as_array().dim();
        let rows = query_shape.0;
        let features = query_shape.1;
        let prototypes = prototype_shape.0;
        let groups = maximum_distance_squared_by_group.as_array().len();
        let query_features = query_features.as_slice()?;
        let query_groups = query_groups.as_slice()?;
        let predicted_component_delta = predicted_component_delta.as_slice()?;
        let predicted_aggregate_delta = predicted_aggregate_delta.as_slice()?;
        let available = candidate_available.as_slice()?;
        let inverse_scale = feature_inverse_scale.as_slice()?;
        let prototype_features = prototype_features.as_slice()?;
        let prototype_groups = prototype_groups.as_slice()?;
        let prototype_component_residuals = prototype_component_residuals.as_slice()?;
        let prototype_aggregate_residuals = prototype_aggregate_residuals.as_slice()?;
        let group_component_lower_extension = group_component_lower_extension.as_slice()?;
        let group_component_upper_extension = group_component_upper_extension.as_slice()?;
        let group_aggregate_lower_extension = group_aggregate_lower_extension.as_slice()?;
        let group_aggregate_upper_extension = group_aggregate_upper_extension.as_slice()?;
        let maximum_distance_squared_by_group = maximum_distance_squared_by_group.as_slice()?;
        let envelope_out = envelope_diagnostics_out.as_slice_mut()?;
        let selection_out = selection_out.as_slice_mut()?;
        let nearest_index_out = nearest_prototype_index_out.as_slice_mut()?;
        let nearest_distance_out = nearest_distance_squared_out.as_slice_mut()?;
        let supported_out = profile_supported_out.as_slice_mut()?;
        let timing_out = timing_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        if rows == 0
            || query_groups.len() != rows
            || predicted_component_shape != (rows, CANDIDATES, COMPONENTS)
            || predicted_aggregate_shape != (rows, CANDIDATES)
            || available_shape != (rows, CANDIDATES)
            || available.iter().any(|value| *value > 1)
            || inverse_scale.len() != features
            || prototype_shape.1 != features
            || prototype_groups.len() != prototypes
            || prototype_component_shape != (prototypes, CANDIDATES, COMPONENTS)
            || prototype_aggregate_shape != (prototypes, CANDIDATES)
            || group_component_lower_shape != (groups, CANDIDATES, COMPONENTS)
            || group_component_upper_shape != (groups, CANDIDATES, COMPONENTS)
            || group_aggregate_lower_shape != (groups, CANDIDATES)
            || group_aggregate_upper_shape != (groups, CANDIDATES)
            || envelope_shape != (rows, CANDIDATES, DIAGNOSTICS)
            || selection_shape != (rows, 6)
            || nearest_index_out.len() != rows
            || nearest_distance_out.len() != rows
            || supported_out.len() != rows
            || timing_out.len() != rows
            || allocation_calls_out.len() != rows
            || allocated_bytes_out.len() != rows
        {
            return Err(PyValueError::new_err(
                "terminal residual prototypes expect query[R,F], group[R], predicted components[R,3,6], predicted aggregate/availability[R,3], inverse scale[F], prototypes[P,F]/groups[P]/components[P,3,6]/aggregate[P,3], group extensions[G,3,6]/[G,3], envelopes[R,3,14], selection[R,6], and scalar outputs[R]",
            ));
        }
        let profile = TerminalImpactResidualPrototypeProfile {
            feature_inverse_scale: inverse_scale,
            prototype_features,
            prototype_groups,
            prototype_component_residuals,
            prototype_aggregate_residuals,
            group_component_lower_extension,
            group_component_upper_extension,
            group_aggregate_lower_extension,
            group_aggregate_upper_extension,
            maximum_distance_squared_by_group,
        }
        .validate()
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid terminal residual prototype profile: {error:?}"
            ))
        })?;
        let query_at = |row: usize| TerminalImpactResidualPrototypeQuery {
            features: &query_features[row * features..(row + 1) * features],
            group: query_groups[row],
            predicted_component_delta: std::array::from_fn(|candidate| {
                std::array::from_fn(|component| {
                    predicted_component_delta
                        [(row * CANDIDATES + candidate) * COMPONENTS + component]
                })
            }),
            predicted_aggregate_delta: std::array::from_fn(|candidate| {
                predicted_aggregate_delta[row * CANDIDATES + candidate]
            }),
            available: std::array::from_fn(|candidate| {
                available[row * CANDIDATES + candidate] != 0
            }),
            maximum_component_regression,
            minimum_component_improvement,
        };

        // Preflight the complete batch before any caller-owned output changes.
        for row in 0..rows {
            profile.score(query_at(row)).map_err(|error| {
                PyValueError::new_err(format!(
                    "invalid terminal residual prototype query at row {row}: {error:?}"
                ))
            })?;
        }
        for row in 0..rows {
            let allocation_before = allocation_snapshot();
            let started = Instant::now();
            let output = profile
                .score(query_at(row))
                .expect("preflighted terminal residual prototype query");
            let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let allocation_after = allocation_snapshot();
            if allocation_after != allocation_before {
                return Err(PyValueError::new_err(
                    "terminal residual prototype query allocated inside the Rust hot path",
                ));
            }
            for candidate in 0..CANDIDATES {
                let offset = (row * CANDIDATES + candidate) * DIAGNOSTICS;
                envelope_out[offset..offset + COMPONENTS]
                    .copy_from_slice(&output.candidates[candidate].component_lower);
                envelope_out[offset + COMPONENTS..offset + 2 * COMPONENTS]
                    .copy_from_slice(&output.candidates[candidate].component_upper);
                envelope_out[offset + 2 * COMPONENTS] =
                    output.candidates[candidate].aggregate_lower;
                envelope_out[offset + 2 * COMPONENTS + 1] =
                    output.candidates[candidate].aggregate_upper;
            }
            let selection_offset = row * 6;
            selection_out[selection_offset..selection_offset + 6].copy_from_slice(&[
                output.selection.selected_index as f64,
                output.selection.baseline_index as f64,
                output.selection.maximum_component_delta_upper,
                output.selection.maximum_guaranteed_component_improvement,
                output.selection.aggregate_delta_lower,
                output.selection.aggregate_delta_upper,
            ]);
            nearest_index_out[row] = output
                .nearest_prototype_index
                .map_or(-1, |index| index as i64);
            nearest_distance_out[row] = output.nearest_distance_squared;
            supported_out[row] = u8::from(output.profile_supported);
            timing_out[row] = elapsed_ns;
            allocation_calls_out[row] = allocation_after.0 - allocation_before.0;
            allocated_bytes_out[row] = allocation_after.1 - allocation_before.1;
        }
        Ok(())
    }

    /// Conservatively score every generalized velocity in a componentwise
    /// box. The output is a pressure upper bound, not a probability-weighted
    /// estimate or a claim that every box corner is dynamically reachable.
    #[allow(clippy::too_many_arguments)]
    fn score_terminal_impact_velocity_box_batch(
        &self,
        root_state: PyReadonlyArray2<'_, f64>,
        root_velocity_lower: PyReadonlyArray2<'_, f64>,
        root_velocity_upper: PyReadonlyArray2<'_, f64>,
        joint_position: PyReadonlyArray1<'_, f64>,
        joint_velocity_lower: PyReadonlyArray2<'_, f64>,
        joint_velocity_upper: PyReadonlyArray2<'_, f64>,
        joint_position_lower: PyReadonlyArray1<'_, f64>,
        joint_position_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray1<'_, u8>,
        root_angular_acceleration: PyReadonlyArray2<'_, f64>,
        joint_acceleration: PyReadonlyArray2<'_, f64>,
        maximum_actuator_effort_utilization: PyReadonlyArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_state_shape = root_state.as_array().dim();
        let root_lower_shape = root_velocity_lower.as_array().dim();
        let root_upper_shape = root_velocity_upper.as_array().dim();
        let joint_lower_shape = joint_velocity_lower.as_array().dim();
        let joint_upper_shape = joint_velocity_upper.as_array().dim();
        let root_acceleration_shape = root_angular_acceleration.as_array().dim();
        let joint_acceleration_shape = joint_acceleration.as_array().dim();
        let diagnostic_shape = diagnostics_out.as_array().dim();
        let root_state = root_state.as_slice()?;
        let root_velocity_lower = root_velocity_lower.as_slice()?;
        let root_velocity_upper = root_velocity_upper.as_slice()?;
        let joint_position = joint_position.as_slice()?;
        let joint_velocity_lower = joint_velocity_lower.as_slice()?;
        let joint_velocity_upper = joint_velocity_upper.as_slice()?;
        let joint_position_lower = joint_position_lower.as_slice()?;
        let joint_position_upper = joint_position_upper.as_slice()?;
        let joint_velocity_limit = joint_velocity_limit.as_slice()?;
        let available = candidate_available.as_slice()?;
        let root_acceleration = root_angular_acceleration.as_slice()?;
        let joint_acceleration = joint_acceleration.as_slice()?;
        let effort = maximum_actuator_effort_utilization.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let rows = root_state_shape.0;
        let joints = joint_position.len();
        if rows == 0
            || rows > 64
            || root_state_shape != (rows, 3)
            || root_lower_shape != (rows, 3)
            || root_upper_shape != (rows, 3)
            || joint_lower_shape != (rows, joints)
            || joint_upper_shape != (rows, joints)
            || joint_position_lower.len() != joints
            || joint_position_upper.len() != joints
            || joint_velocity_limit.len() != joints
            || available.len() != rows
            || available.iter().any(|value| *value > 1)
            || root_acceleration_shape != (rows, 2)
            || joint_acceleration_shape != (rows, joints)
            || effort.len() != rows
            || diagnostic_shape != (rows, 17)
        {
            return Err(PyValueError::new_err(
                "terminal velocity box expects root state[N,3], lower/upper root velocity[N,3], common joint position/limits[J], lower/upper joint velocity and acceleration[N,J], availability/effort[N], root acceleration[N,2], and diagnostics[N,17] for 1<=N<=64",
            ));
        }
        let config = TerminalImpactConfig::default();
        let state_at = |row: usize| TerminalImpactVelocityBoxState {
            root_clearance_m: root_state[3 * row],
            root_vertical_velocity_lower_m_s: root_velocity_lower[3 * row],
            root_vertical_velocity_upper_m_s: root_velocity_upper[3 * row],
            root_tilt_rad: [root_state[3 * row + 1], root_state[3 * row + 2]],
            root_angular_rate_lower_rad_s: [
                root_velocity_lower[3 * row + 1],
                root_velocity_lower[3 * row + 2],
            ],
            root_angular_rate_upper_rad_s: [
                root_velocity_upper[3 * row + 1],
                root_velocity_upper[3 * row + 2],
            ],
            joint_position_rad: joint_position,
            joint_velocity_lower_rad_s: &joint_velocity_lower[row * joints..(row + 1) * joints],
            joint_velocity_upper_rad_s: &joint_velocity_upper[row * joints..(row + 1) * joints],
            joint_position_lower_rad: joint_position_lower,
            joint_position_upper_rad: joint_position_upper,
            joint_velocity_limit_rad_s: joint_velocity_limit,
        };
        let candidate_at = |row: usize| TerminalImpactCandidate {
            available: available[row] != 0,
            root_angular_acceleration_rad_s2: [
                root_acceleration[2 * row],
                root_acceleration[2 * row + 1],
            ],
            joint_acceleration_rad_s2: &joint_acceleration[row * joints..(row + 1) * joints],
            maximum_actuator_effort_utilization: effort[row],
        };
        // Full preflight preserves caller output on any late invalid row.
        for row in 0..rows {
            score_terminal_impact_velocity_box_upper(state_at(row), candidate_at(row), config)
                .map_err(|error| {
                    PyValueError::new_err(format!(
                        "invalid terminal-impact velocity box row {row}: {error:?}"
                    ))
                })?;
        }

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for row in 0..rows {
            let score =
                score_terminal_impact_velocity_box_upper(state_at(row), candidate_at(row), config)
                    .expect("validated terminal-impact velocity box row");
            write_terminal_impact_score_diagnostics(
                score,
                &mut diagnostics[row * 17..(row + 1) * 17],
            );
        }
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "terminal-impact velocity box batch allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Score exactly three componentwise terminal velocity boxes and apply
    /// the existing conservative componentwise selector in the same timed,
    /// allocation-free Rust boundary. This selects an evaluation candidate;
    /// it does not admit a plant command or authority.
    #[allow(clippy::too_many_arguments)]
    fn select_terminal_impact_velocity_box_candidates(
        &self,
        root_state: PyReadonlyArray1<'_, f64>,
        root_velocity_lower: PyReadonlyArray2<'_, f64>,
        root_velocity_upper: PyReadonlyArray2<'_, f64>,
        joint_position: PyReadonlyArray1<'_, f64>,
        joint_velocity_lower: PyReadonlyArray2<'_, f64>,
        joint_velocity_upper: PyReadonlyArray2<'_, f64>,
        joint_position_lower: PyReadonlyArray1<'_, f64>,
        joint_position_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray1<'_, u8>,
        root_angular_acceleration: PyReadonlyArray2<'_, f64>,
        joint_acceleration: PyReadonlyArray2<'_, f64>,
        maximum_actuator_effort_utilization: PyReadonlyArray1<'_, f64>,
        baseline_index: usize,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut diagnostics_out: PyReadwriteArray2<'_, f64>,
        mut selection_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        let root_lower_shape = root_velocity_lower.as_array().dim();
        let root_upper_shape = root_velocity_upper.as_array().dim();
        let joint_lower_shape = joint_velocity_lower.as_array().dim();
        let joint_upper_shape = joint_velocity_upper.as_array().dim();
        let root_acceleration_shape = root_angular_acceleration.as_array().dim();
        let joint_acceleration_shape = joint_acceleration.as_array().dim();
        let diagnostic_shape = diagnostics_out.as_array().dim();
        let root_state = root_state.as_slice()?;
        let root_velocity_lower = root_velocity_lower.as_slice()?;
        let root_velocity_upper = root_velocity_upper.as_slice()?;
        let joint_position = joint_position.as_slice()?;
        let joint_velocity_lower = joint_velocity_lower.as_slice()?;
        let joint_velocity_upper = joint_velocity_upper.as_slice()?;
        let joint_position_lower = joint_position_lower.as_slice()?;
        let joint_position_upper = joint_position_upper.as_slice()?;
        let joint_velocity_limit = joint_velocity_limit.as_slice()?;
        let available = candidate_available.as_slice()?;
        let root_acceleration = root_angular_acceleration.as_slice()?;
        let joint_acceleration = joint_acceleration.as_slice()?;
        let effort = maximum_actuator_effort_utilization.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let selection_out = selection_out.as_slice_mut()?;
        let joints = joint_position.len();
        if root_state.len() != 3
            || root_lower_shape != (CANDIDATES, 3)
            || root_upper_shape != (CANDIDATES, 3)
            || joint_lower_shape != (CANDIDATES, joints)
            || joint_upper_shape != (CANDIDATES, joints)
            || joint_position_lower.len() != joints
            || joint_position_upper.len() != joints
            || joint_velocity_limit.len() != joints
            || available.len() != CANDIDATES
            || available.iter().any(|value| *value > 1)
            || root_acceleration_shape != (CANDIDATES, 2)
            || joint_acceleration_shape != (CANDIDATES, joints)
            || effort.len() != CANDIDATES
            || diagnostic_shape != (CANDIDATES, 17)
            || selection_out.len() != 6
        {
            return Err(PyValueError::new_err(
                "terminal velocity-box selection expects root state[3], lower/upper root velocity[3,3], common joint position/limits[J], lower/upper joint velocity and acceleration[3,J], availability/effort[3], root acceleration[3,2], diagnostics[3,17], and selection[6]",
            ));
        }
        let config = TerminalImpactConfig::default();
        let state_at = |candidate: usize| TerminalImpactVelocityBoxState {
            root_clearance_m: root_state[0],
            root_vertical_velocity_lower_m_s: root_velocity_lower[3 * candidate],
            root_vertical_velocity_upper_m_s: root_velocity_upper[3 * candidate],
            root_tilt_rad: [root_state[1], root_state[2]],
            root_angular_rate_lower_rad_s: [
                root_velocity_lower[3 * candidate + 1],
                root_velocity_lower[3 * candidate + 2],
            ],
            root_angular_rate_upper_rad_s: [
                root_velocity_upper[3 * candidate + 1],
                root_velocity_upper[3 * candidate + 2],
            ],
            joint_position_rad: joint_position,
            joint_velocity_lower_rad_s: &joint_velocity_lower
                [candidate * joints..(candidate + 1) * joints],
            joint_velocity_upper_rad_s: &joint_velocity_upper
                [candidate * joints..(candidate + 1) * joints],
            joint_position_lower_rad: joint_position_lower,
            joint_position_upper_rad: joint_position_upper,
            joint_velocity_limit_rad_s: joint_velocity_limit,
        };
        let candidate_at = |candidate: usize| TerminalImpactCandidate {
            available: available[candidate] != 0,
            root_angular_acceleration_rad_s2: [
                root_acceleration[2 * candidate],
                root_acceleration[2 * candidate + 1],
            ],
            joint_acceleration_rad_s2: &joint_acceleration
                [candidate * joints..(candidate + 1) * joints],
            maximum_actuator_effort_utilization: effort[candidate],
        };
        let mut scores = [TerminalImpactScore::default(); CANDIDATES];
        for candidate in 0..CANDIDATES {
            scores[candidate] = score_terminal_impact_velocity_box_upper(
                state_at(candidate),
                candidate_at(candidate),
                config,
            )
            .map_err(|error| {
                PyValueError::new_err(format!(
                    "invalid terminal-impact velocity box candidate {candidate}: {error:?}"
                ))
            })?;
        }
        select_conservative_terminal_impact_candidate(
            &scores,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid terminal-impact velocity box selection: {error:?}"
            ))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for candidate in 0..CANDIDATES {
            scores[candidate] = score_terminal_impact_velocity_box_upper(
                state_at(candidate),
                candidate_at(candidate),
                config,
            )
            .expect("validated terminal-impact velocity box candidate");
        }
        let selection = select_conservative_terminal_impact_candidate(
            &scores,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .expect("validated terminal-impact velocity box selection");
        for candidate in 0..CANDIDATES {
            write_terminal_impact_score_diagnostics(
                scores[candidate],
                &mut diagnostics[candidate * 17..(candidate + 1) * 17],
            );
        }
        selection_out.copy_from_slice(&[
            selection.selected_index as f64,
            selection.baseline_index as f64,
            selection.selected_score,
            selection.baseline_score,
            selection.maximum_component_regression,
            selection.maximum_component_improvement,
        ]);
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "terminal-impact velocity box selection allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Select exactly three paired candidate-minus-baseline terminal pressure
    /// boxes. The six columns are tilt, angular-rate, joint-position,
    /// joint-velocity, actuator-effort pressure, and raw joint-headroom loss
    /// deltas. Impact speed is candidate-invariant and availability is the
    /// admission gate. This boundary is allocation-free and atomic, but
    /// remains an evaluation selector rather than command authority.
    #[allow(clippy::too_many_arguments)]
    fn select_terminal_impact_component_delta_box_candidates(
        &self,
        component_lower: PyReadonlyArray2<'_, f64>,
        component_upper: PyReadonlyArray2<'_, f64>,
        aggregate_lower: PyReadonlyArray1<'_, f64>,
        aggregate_upper: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray1<'_, u8>,
        baseline_index: usize,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut diagnostics_out: PyReadwriteArray2<'_, f64>,
        mut selection_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        const DIAGNOSTICS: usize = 2 * TERMINAL_IMPACT_PAIRED_COMPONENTS + 2;
        let lower_shape = component_lower.as_array().dim();
        let upper_shape = component_upper.as_array().dim();
        let diagnostic_shape = diagnostics_out.as_array().dim();
        let component_lower = component_lower.as_slice()?;
        let component_upper = component_upper.as_slice()?;
        let aggregate_lower = aggregate_lower.as_slice()?;
        let aggregate_upper = aggregate_upper.as_slice()?;
        let available = candidate_available.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let selection_out = selection_out.as_slice_mut()?;
        if lower_shape != (CANDIDATES, TERMINAL_IMPACT_PAIRED_COMPONENTS)
            || upper_shape != (CANDIDATES, TERMINAL_IMPACT_PAIRED_COMPONENTS)
            || aggregate_lower.len() != CANDIDATES
            || aggregate_upper.len() != CANDIDATES
            || available.len() != CANDIDATES
            || available.iter().any(|value| *value > 1)
            || diagnostic_shape != (CANDIDATES, DIAGNOSTICS)
            || selection_out.len() != 6
        {
            return Err(PyValueError::new_err(
                "paired terminal delta selection expects component lower/upper[3,6], aggregate lower/upper and availability[3], diagnostics[3,14], and selection[6]",
            ));
        }
        let candidates: [TerminalImpactComponentDeltaBox; CANDIDATES] =
            std::array::from_fn(|candidate| {
                let offset = candidate * TERMINAL_IMPACT_PAIRED_COMPONENTS;
                TerminalImpactComponentDeltaBox {
                    available: available[candidate] != 0,
                    component_lower: std::array::from_fn(|component| {
                        component_lower[offset + component]
                    }),
                    component_upper: std::array::from_fn(|component| {
                        component_upper[offset + component]
                    }),
                    aggregate_lower: aggregate_lower[candidate],
                    aggregate_upper: aggregate_upper[candidate],
                }
            });
        select_conservative_terminal_impact_delta_candidate(
            &candidates,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid paired terminal-impact delta selection: {error:?}"
            ))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let selection = select_conservative_terminal_impact_delta_candidate(
            &candidates,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .expect("validated paired terminal-impact delta selection");
        for (candidate, values) in diagnostics.chunks_exact_mut(DIAGNOSTICS).enumerate() {
            values[..TERMINAL_IMPACT_PAIRED_COMPONENTS]
                .copy_from_slice(&candidates[candidate].component_lower);
            values[TERMINAL_IMPACT_PAIRED_COMPONENTS..2 * TERMINAL_IMPACT_PAIRED_COMPONENTS]
                .copy_from_slice(&candidates[candidate].component_upper);
            values[2 * TERMINAL_IMPACT_PAIRED_COMPONENTS] = candidates[candidate].aggregate_lower;
            values[2 * TERMINAL_IMPACT_PAIRED_COMPONENTS + 1] =
                candidates[candidate].aggregate_upper;
        }
        selection_out.copy_from_slice(&[
            selection.selected_index as f64,
            selection.baseline_index as f64,
            selection.maximum_component_delta_upper,
            selection.maximum_guaranteed_component_improvement,
            selection.aggregate_delta_lower,
            selection.aggregate_delta_upper,
        ]);
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "paired terminal-impact delta selection allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Envelope generalized velocity jumps over an explicitly enumerated
    /// finite contact-estimator hypothesis set. This is not a continuous-set
    /// certificate between the caller's hypotheses.
    #[allow(clippy::too_many_arguments)]
    fn coupled_contact_hypothesis_velocity_envelope(
        &mut self,
        contact_velocity_hypotheses: PyReadonlyArray3<'_, f64>,
        delassus: PyReadonlyArray2<'_, f64>,
        impulse_upper_hypotheses: PyReadonlyArray3<'_, f64>,
        friction_hypotheses: PyReadonlyArray2<'_, f64>,
        restitution_hypotheses: PyReadonlyArray1<'_, f64>,
        diagonal_regularization_ratio_hypotheses: PyReadonlyArray1<'_, f64>,
        impulse_velocity_response: PyReadonlyArray3<'_, f64>,
        sweeps: usize,
        mut generalized_velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut generalized_velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let velocity_shape = contact_velocity_hypotheses.as_array().dim();
        let delassus_shape = delassus.as_array().dim();
        let upper_shape = impulse_upper_hypotheses.as_array().dim();
        let friction_shape = friction_hypotheses.as_array().dim();
        let response_shape = impulse_velocity_response.as_array().dim();
        let contact_velocity_hypotheses = contact_velocity_hypotheses.as_slice()?;
        let delassus = delassus.as_slice()?;
        let impulse_upper_hypotheses = impulse_upper_hypotheses.as_slice()?;
        let friction_hypotheses = friction_hypotheses.as_slice()?;
        let restitution_hypotheses = restitution_hypotheses.as_slice()?;
        let diagonal_regularization_ratio_hypotheses =
            diagonal_regularization_ratio_hypotheses.as_slice()?;
        let impulse_velocity_response = impulse_velocity_response.as_slice()?;
        let generalized_velocity_lower_out = generalized_velocity_lower_out.as_slice_mut()?;
        let generalized_velocity_upper_out = generalized_velocity_upper_out.as_slice_mut()?;
        let hypotheses = restitution_hypotheses.len();
        let contacts = self.point_specs.len();
        let axes = contacts * CONTACT_TRANSITION_IMPULSE_WIDTH;
        let generalized_dof = self.program.model.dof + 6;
        if velocity_shape != (hypotheses, contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || upper_shape != velocity_shape
            || friction_shape != (hypotheses, contacts)
            || diagonal_regularization_ratio_hypotheses.len() != hypotheses
            || delassus_shape != (axes, axes)
            || response_shape != (generalized_dof, contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || generalized_velocity_lower_out.len() != generalized_dof
            || generalized_velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "contact hypothesis envelope expects velocity/upper[H,{contacts},3], friction[H,{contacts}], restitution/regularization[H], delassus[{axes},{axes}], response[{generalized_dof},{contacts},3], and lower/upper[{generalized_dof}]"
            )));
        }
        let input = CoupledContactHypothesisEnvelopeInput {
            hypothesis_count: hypotheses,
            contact_velocity_hypotheses,
            delassus,
            impulse_upper_hypotheses,
            friction_hypotheses,
            restitution_hypotheses,
            diagonal_regularization_ratio_hypotheses,
            impulse_velocity_response,
            sweeps,
        };
        write_coupled_contact_hypothesis_velocity_envelope(
            input,
            &mut self.coupled_impulse_scratch,
            &mut self.coupled_velocity_scratch,
            &mut self.coupled_delta_scratch,
            generalized_velocity_lower_out,
            generalized_velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid contact hypothesis envelope: {error:?}"))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_coupled_contact_hypothesis_velocity_envelope(
            input,
            &mut self.coupled_impulse_scratch,
            &mut self.coupled_velocity_scratch,
            &mut self.coupled_delta_scratch,
            generalized_velocity_lower_out,
            generalized_velocity_upper_out,
        )
        .expect("validated contact hypothesis envelope");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "contact hypothesis envelope allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Derive arbitrary-frame point responses and the complete coupled
    /// Delassus operator. Contacts follow the construction-time frame order;
    /// repeated frame names are valid for multiple points on one rigid body.
    #[allow(clippy::too_many_arguments)]
    fn point_impulse_velocity_response(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        contact_points_world: PyReadonlyArray2<'_, f64>,
        contact_bases_world: PyReadonlyArray3<'_, f64>,
        mut response_out: PyReadwriteArray3<'_, f64>,
        mut effective_mass_out: PyReadwriteArray2<'_, f64>,
        mut delassus_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let point_shape = contact_points_world.as_array().dim();
        let basis_shape = contact_bases_world.as_array().dim();
        let response_shape = response_out.as_array().dim();
        let effective_mass_shape = effective_mass_out.as_array().dim();
        let delassus_shape = delassus_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let contact_points_world = contact_points_world.as_slice()?;
        let contact_bases_world = contact_bases_world.as_slice()?;
        let response_out = response_out.as_slice_mut()?;
        let effective_mass_out = effective_mass_out.as_slice_mut()?;
        let delassus_out = delassus_out.as_slice_mut()?;
        let contacts = self.point_specs.len();
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        let contact_axes = contacts * CONTACT_TRANSITION_IMPULSE_WIDTH;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || point_shape != (contacts, 3)
            || basis_shape != (contacts, 3, 3)
            || response_shape != (generalized_dof, contacts, 3)
            || effective_mass_shape != (contacts, 3)
            || delassus_shape != (contact_axes, contact_axes)
        {
            return Err(PyValueError::new_err(format!(
                "point response expects root[3], quaternion[4], q[{dof}], points[{contacts},3], bases[{contacts},3,3], response[{generalized_dof},{contacts},3], effective_mass[{contacts},3], and delassus[{contact_axes},{contact_axes}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(contact_points_world)
            .chain(contact_bases_world)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "point response inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("point response quaternion is degenerate"))?;
        for (contact, spec) in self.point_specs.iter_mut().enumerate() {
            spec.point_world = Vec3::new(
                contact_points_world[contact * 3],
                contact_points_world[contact * 3 + 1],
                contact_points_world[contact * 3 + 2],
            );
            spec.basis_world = std::array::from_fn(|axis| {
                let start = contact * 9 + axis * 3;
                Vec3::new(
                    contact_bases_world[start],
                    contact_bases_world[start + 1],
                    contact_bases_world[start + 2],
                )
            });
        }
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_point_impulse_velocity_response_with_delassus(
            &self.program.model,
            &self.robot,
            &self.point_specs,
            &mut self.scratch,
            response_out,
            effective_mass_out,
            delassus_out,
        )
        .map_err(|error| PyValueError::new_err(format!("invalid point response: {error:?}")))?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_point_impulse_velocity_response_with_delassus(
            &self.program.model,
            &self.robot,
            &self.point_specs,
            &mut self.scratch,
            response_out,
            effective_mass_out,
            delassus_out,
        )
        .expect("validated point response");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "point response allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Derive arbitrary-frame six-axis wrench responses and their complete
    /// coupled Delassus operator. Wrench columns are moment XYZ then force XYZ
    /// about the caller-declared reference point.
    #[allow(clippy::too_many_arguments)]
    fn spatial_impulse_velocity_response(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        reference_points_world: PyReadonlyArray2<'_, f64>,
        wrench_bases_world: PyReadonlyArray3<'_, f64>,
        mut response_out: PyReadwriteArray3<'_, f64>,
        mut delassus_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let point_shape = reference_points_world.as_array().dim();
        let basis_shape = wrench_bases_world.as_array().dim();
        let response_shape = response_out.as_array().dim();
        let delassus_shape = delassus_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let reference_points_world = reference_points_world.as_slice()?;
        let wrench_bases_world = wrench_bases_world.as_slice()?;
        let response_out = response_out.as_slice_mut()?;
        let delassus_out = delassus_out.as_slice_mut()?;
        let contacts = self.spatial_specs.len();
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        let spatial_axes = contacts * SPATIAL_IMPULSE_WIDTH;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || point_shape != (contacts, 3)
            || basis_shape != (contacts, 3, 3)
            || response_shape != (generalized_dof, contacts, SPATIAL_IMPULSE_WIDTH)
            || delassus_shape != (spatial_axes, spatial_axes)
        {
            return Err(PyValueError::new_err(format!(
                "spatial response expects root[3], quaternion[4], q[{dof}], reference points[{contacts},3], bases[{contacts},3,3], response[{generalized_dof},{contacts},6], and delassus[{spatial_axes},{spatial_axes}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(reference_points_world)
            .chain(wrench_bases_world)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "spatial response inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("spatial response quaternion is degenerate"))?;
        for (contact, spec) in self.spatial_specs.iter_mut().enumerate() {
            spec.reference_point_world = Vec3::new(
                reference_points_world[contact * 3],
                reference_points_world[contact * 3 + 1],
                reference_points_world[contact * 3 + 2],
            );
            spec.basis_world = std::array::from_fn(|axis| {
                let start = contact * 9 + axis * 3;
                Vec3::new(
                    wrench_bases_world[start],
                    wrench_bases_world[start + 1],
                    wrench_bases_world[start + 2],
                )
            });
        }
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_spatial_impulse_velocity_response(
            &self.program.model,
            &self.robot,
            &self.spatial_specs,
            &mut self.scratch,
            response_out,
            delassus_out,
        )
        .map_err(|error| PyValueError::new_err(format!("invalid spatial response: {error:?}")))?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_spatial_impulse_velocity_response(
            &self.program.model,
            &self.robot,
            &self.spatial_specs,
            &mut self.scratch,
            response_out,
            delassus_out,
        )
        .expect("validated spatial response");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "spatial response allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Compute `M(q) * (observed_delta - predicted_delta)` for each candidate.
    #[allow(clippy::too_many_arguments)]
    fn generalized_momentum_impulse_residuals(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        observed_delta_velocity: PyReadonlyArray1<'_, f64>,
        predicted_delta_velocity: PyReadonlyArray2<'_, f64>,
        mut residual_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let predicted_shape = predicted_delta_velocity.as_array().dim();
        let residual_shape = residual_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let observed_delta_velocity = observed_delta_velocity.as_slice()?;
        let predicted_delta_velocity = predicted_delta_velocity.as_slice()?;
        let residual_out = residual_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || observed_delta_velocity.len() != generalized_dof
            || predicted_shape.0 == 0
            || predicted_shape.1 != generalized_dof
            || residual_shape != predicted_shape
        {
            return Err(PyValueError::new_err(format!(
                "generalized momentum residual expects root[3], quaternion[4], q[{dof}], observed[{generalized_dof}], predicted[C,{generalized_dof}], and residual[C,{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(observed_delta_velocity)
            .chain(predicted_delta_velocity)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "generalized momentum residual inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| {
            PyValueError::new_err("generalized momentum residual quaternion is degenerate")
        })?;
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_momentum_impulse_residuals(
            &self.program.model,
            &self.robot,
            observed_delta_velocity,
            predicted_delta_velocity,
            &mut self.scratch,
            residual_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid generalized momentum residual: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_momentum_impulse_residuals(
            &self.program.model,
            &self.robot,
            observed_delta_velocity,
            predicted_delta_velocity,
            &mut self.scratch,
            residual_out,
        )
        .expect("validated generalized momentum residual");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "generalized momentum residual allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Map a generalized-momentum impulse box through the exact inverse mass.
    #[allow(clippy::too_many_arguments)]
    fn generalized_velocity_interval_from_momentum_box(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        momentum_lower: PyReadonlyArray1<'_, f64>,
        momentum_upper: PyReadonlyArray1<'_, f64>,
        mut velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let momentum_lower = momentum_lower.as_slice()?;
        let momentum_upper = momentum_upper.as_slice()?;
        let velocity_lower_out = velocity_lower_out.as_slice_mut()?;
        let velocity_upper_out = velocity_upper_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || momentum_lower.len() != generalized_dof
            || momentum_upper.len() != generalized_dof
            || velocity_lower_out.len() != generalized_dof
            || velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "momentum box expects root[3], quaternion[4], q[{dof}], momentum lower/upper[{generalized_dof}], and velocity lower/upper[{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "momentum-box model state must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("momentum-box quaternion is degenerate"))?;
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_velocity_interval_from_momentum_box(
            &self.program.model,
            &self.robot,
            momentum_lower,
            momentum_upper,
            &mut self.scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .map_err(|error| PyValueError::new_err(format!("invalid momentum box: {error:?}")))?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_velocity_interval_from_momentum_box(
            &self.program.model,
            &self.robot,
            momentum_lower,
            momentum_upper,
            &mut self.scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .expect("validated momentum box");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "momentum-box projection allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Project a zero-centered generalized-impulse ellipsoid
    /// `pᵀM⁻¹p <= twice_energy` to exact componentwise velocity bounds.
    #[allow(clippy::too_many_arguments)]
    fn generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        twice_kinetic_energy_upper_j: f64,
        mut velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let velocity_lower_out = velocity_lower_out.as_slice_mut()?;
        let velocity_upper_out = velocity_upper_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || velocity_lower_out.len() != generalized_dof
            || velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "kinetic impulse ellipsoid expects root[3], quaternion[4], q[{dof}], and velocity lower/upper[{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "kinetic impulse ellipsoid model state must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| {
            PyValueError::new_err("kinetic impulse ellipsoid quaternion is degenerate")
        })?;
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
            &self.program.model,
            &self.robot,
            twice_kinetic_energy_upper_j,
            &mut self.scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid kinetic impulse ellipsoid: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
            &self.program.model,
            &self.robot,
            twice_kinetic_energy_upper_j,
            &mut self.scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .expect("validated kinetic impulse ellipsoid");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "kinetic impulse ellipsoid allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Project separately budgeted root and articulated generalized-impulse
    /// ellipsoids to exact componentwise velocity bounds. Their supports add,
    /// while both energy witnesses remain independently visible.
    #[allow(clippy::too_many_arguments)]
    fn generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        root_twice_kinetic_energy_upper_j: f64,
        articulated_twice_kinetic_energy_upper_j: f64,
        mut velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let velocity_lower_out = velocity_lower_out.as_slice_mut()?;
        let velocity_upper_out = velocity_upper_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || velocity_lower_out.len() != generalized_dof
            || velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "split kinetic impulse ellipsoids expect root[3], quaternion[4], q[{dof}], and velocity lower/upper[{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "split kinetic impulse ellipsoid model state must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| {
            PyValueError::new_err("split kinetic impulse ellipsoid quaternion is degenerate")
        })?;
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
            &self.program.model,
            &self.robot,
            root_twice_kinetic_energy_upper_j,
            articulated_twice_kinetic_energy_upper_j,
            &mut self.scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid split kinetic impulse ellipsoids: {error:?}"
            ))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
            &self.program.model,
            &self.robot,
            root_twice_kinetic_energy_upper_j,
            articulated_twice_kinetic_energy_upper_j,
            &mut self.scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .expect("validated split kinetic impulse ellipsoids");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "split kinetic impulse ellipsoid allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }
}

/// Stateful Upkie example-policy boundary used by closed-loop plant evals.
/// Python owns plant state and experiment sequencing; this Rust session owns
/// the exact reference PI state, coordinate/sign compilation, clamps, and
/// wheel-acceleration lowering without per-step allocation.
#[pyclass(unsendable)]
struct UpkieBalanceSession {
    program: MotionProgram,
    balancer: UpkieWheelBalancer,
    state: UpkieWheelBalancerState,
    joint_velocity: Vec<f64>,
    robot: RobotState,
    cache: ModelCache,
    ik_scratch: PlanarIkScratch,
    support_frames: [FrameId; 2],
    leg_coordinates: [[usize; 2]; 2],
    capture_config: UpkieCaptureReferenceConfig,
    capture_state: UpkieCaptureReferenceState,
    planar_config: UpkiePlanarCaptureConfig,
    planar_state: UpkiePlanarCaptureState,
    lateral_viability_config: UpkieLateralViabilityConfig,
    lateral_viability_state: UpkieLateralViabilityState,
    fall_safe_config: UpkieFallSafeConfig,
    fall_safe_state: UpkieFallSafeState,
    support_contingency_config: SupportContingencyConfig,
    contact_observation_config: ContactObservationConfig,
    contact_observation_state: ContactObservationState<2>,
    contact_command_lease_config: ContactCommandLeaseConfig,
    contact_command_lease_state: ContactCommandLeaseState<2, 6>,
    contact_program_authority_config: ContactProgramAuthorityConfig<2>,
    contact_program_authority_state: ContactProgramAuthorityState<2, 6>,
    viability_request_config: ViabilityRequestConfig<3>,
    viability_request_state: ViabilityRequestState<3>,
    viability_confirmation_config: ViabilityConfirmationConfig<3>,
    viability_confirmation_state: ViabilityConfirmationState<3>,
    viability_hybrid_guard_config: ViabilityHybridGuardConfig,
    viability_hybrid_guard_state: ViabilityHybridGuardState,
    viability_execution_monitor_config: ViabilityExecutionMonitorConfig,
    viability_execution_monitor_state: ViabilityExecutionMonitorState,
    viability_forecast_config: ViabilityForecastConfig,
    viability_poll_state: ViabilityPollState,
    capture_dynamics: DynamicsCache,
    contact_transition_response_scratch: ContactTransitionResponseScratch,
    center_of_mass_jacobian: DMatrix<f64>,
    wheel_jacobians: [DMatrix<f64>; 2],
    atlas: CompiledFrameAtlas,
    external_frames: ExternalFrameInputs,
    atlas_snapshot: FrameAtlasSnapshot,
}

#[pymethods]
impl UpkieBalanceSession {
    #[new]
    fn new(urdf_path: &str) -> PyResult<Self> {
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let robot = RobotState::zeros(&program.model);
        let balancer = UpkieWheelBalancer::compile(&program.model, &robot).map_err(value_error)?;
        let required_frame = |name: &str| {
            program
                .model
                .frame_id(name)
                .ok_or_else(|| PyValueError::new_err(format!("Upkie frame {name} is required")))
        };
        let required_coordinate = |name: &str| {
            let joint = program
                .model
                .joint_id(name)
                .ok_or_else(|| PyValueError::new_err(format!("Upkie joint {name} is required")))?;
            program.model.joints[joint.0].coordinate.ok_or_else(|| {
                PyValueError::new_err(format!("Upkie joint {name} must be actuated"))
            })
        };
        let support_frames = [
            required_frame("left_wheel_center")?,
            required_frame("right_wheel_center")?,
        ];
        let leg_coordinates = [
            [
                required_coordinate("left_hip")?,
                required_coordinate("left_knee")?,
            ],
            [
                required_coordinate("right_hip")?,
                required_coordinate("right_knee")?,
            ],
        ];
        let cache = ModelCache::new(&program.model);
        let ik_scratch = PlanarIkScratch::new(&program.model);
        let dof = program.model.dof;
        let generalized_dof = dof + 6;
        let capture_dynamics = DynamicsCache::new(&program.model);
        let contact_transition_response_scratch =
            ContactTransitionResponseScratch::new(&program.model);
        let center_of_mass_jacobian = DMatrix::zeros(3, generalized_dof);
        let wheel_jacobians = [
            DMatrix::zeros(3, generalized_dof),
            DMatrix::zeros(3, generalized_dof),
        ];
        let atlas = CompiledFrameAtlas::standard(&program.model);
        let external_frames = ExternalFrameInputs::new(&atlas);
        let atlas_snapshot = FrameAtlasSnapshot::new(&atlas);
        Ok(Self {
            program,
            balancer,
            state: UpkieWheelBalancerState::default(),
            joint_velocity: vec![0.0; dof],
            robot,
            cache,
            ik_scratch,
            support_frames,
            leg_coordinates,
            capture_config: UpkieCaptureReferenceConfig::default(),
            capture_state: UpkieCaptureReferenceState::default(),
            planar_config: UpkiePlanarCaptureConfig::default(),
            planar_state: UpkiePlanarCaptureState::default(),
            lateral_viability_config: UpkieLateralViabilityConfig::default(),
            lateral_viability_state: UpkieLateralViabilityState::default(),
            fall_safe_config: UpkieFallSafeConfig::default(),
            fall_safe_state: UpkieFallSafeState::default(),
            support_contingency_config: SupportContingencyConfig::default(),
            contact_observation_config: ContactObservationConfig::default(),
            contact_observation_state: ContactObservationState::default(),
            contact_command_lease_config: ContactCommandLeaseConfig::default(),
            contact_command_lease_state: ContactCommandLeaseState::default(),
            contact_program_authority_config: ContactProgramAuthorityConfig::default(),
            contact_program_authority_state: ContactProgramAuthorityState::default(),
            viability_request_config: ViabilityRequestConfig::default(),
            viability_request_state: ViabilityRequestState::default(),
            viability_confirmation_config: ViabilityConfirmationConfig::default(),
            viability_confirmation_state: ViabilityConfirmationState::default(),
            viability_hybrid_guard_config: ViabilityHybridGuardConfig::default(),
            viability_hybrid_guard_state: ViabilityHybridGuardState::default(),
            viability_execution_monitor_config: ViabilityExecutionMonitorConfig::default(),
            viability_execution_monitor_state: ViabilityExecutionMonitorState::default(),
            viability_forecast_config: ViabilityForecastConfig::default(),
            viability_poll_state: ViabilityPollState::default(),
            capture_dynamics,
            contact_transition_response_scratch,
            center_of_mass_jacobian,
            wheel_jacobians,
            atlas,
            external_frames,
            atlas_snapshot,
        })
    }

    #[getter]
    fn coordinates(&self) -> [usize; 2] {
        self.balancer.coordinates
    }

    #[getter]
    fn integral_velocity(&self) -> f64 {
        self.state.integral_velocity
    }

    #[getter]
    fn capture_velocity_fraction(&self) -> f64 {
        self.capture_config.capture_velocity_fraction
    }

    #[setter]
    fn set_capture_velocity_fraction(&mut self, value: f64) -> PyResult<()> {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {
            return Err(PyValueError::new_err(
                "capture_velocity_fraction must be finite and inside [0, 1]",
            ));
        }
        self.capture_config.capture_velocity_fraction = value;
        self.planar_config.sagittal.capture_velocity_fraction = value;
        Ok(())
    }

    fn reset(&mut self) {
        self.state = UpkieWheelBalancerState::default();
        self.capture_state = UpkieCaptureReferenceState::default();
        self.planar_state = UpkiePlanarCaptureState::default();
        self.lateral_viability_state = UpkieLateralViabilityState::default();
        self.fall_safe_state = UpkieFallSafeState::default();
        self.contact_observation_state = ContactObservationState::default();
        self.contact_command_lease_state = ContactCommandLeaseState::default();
        self.contact_program_authority_state = ContactProgramAuthorityState::default();
        self.viability_request_state = ViabilityRequestState::default();
        self.viability_confirmation_state = ViabilityConfirmationState::default();
        self.viability_hybrid_guard_state = ViabilityHybridGuardState::default();
        self.viability_execution_monitor_state = ViabilityExecutionMonitorState::default();
        self.viability_poll_state = ViabilityPollState::default();
        self.joint_velocity.fill(0.0);
    }

    #[allow(clippy::too_many_arguments)]
    fn configure_planar_capture(
        &mut self,
        steering_release_capture_error_m: f64,
        steering_full_capture_error_m: f64,
        heading_gain_rad_s_per_rad: f64,
        maximum_yaw_rate_rad_s: f64,
        yaw_rate_slew_rad_s2: f64,
        yaw_rate_tracking_gain_per_s: f64,
        maximum_yaw_acceleration_rad_s2: f64,
    ) -> PyResult<()> {
        let values = [
            steering_release_capture_error_m,
            steering_full_capture_error_m,
            heading_gain_rad_s_per_rad,
            maximum_yaw_rate_rad_s,
            yaw_rate_slew_rad_s2,
            yaw_rate_tracking_gain_per_s,
            maximum_yaw_acceleration_rad_s2,
        ];
        if values.iter().any(|value| !value.is_finite())
            || steering_release_capture_error_m < 0.0
            || steering_release_capture_error_m >= steering_full_capture_error_m
            || heading_gain_rad_s_per_rad < 0.0
            || maximum_yaw_rate_rad_s <= 0.0
            || yaw_rate_slew_rad_s2 <= 0.0
            || yaw_rate_tracking_gain_per_s <= 0.0
            || maximum_yaw_acceleration_rad_s2 <= 0.0
        {
            return Err(PyValueError::new_err(
                "planar capture configuration must be finite with ordered positive bounds",
            ));
        }
        self.planar_config.steering_release_capture_error_m = steering_release_capture_error_m;
        self.planar_config.steering_full_capture_error_m = steering_full_capture_error_m;
        self.planar_config.heading_gain_rad_s_per_rad = heading_gain_rad_s_per_rad;
        self.planar_config.maximum_yaw_rate_rad_s = maximum_yaw_rate_rad_s;
        self.planar_config.yaw_rate_slew_rad_s2 = yaw_rate_slew_rad_s2;
        self.planar_config.yaw_rate_tracking_gain_per_s = yaw_rate_tracking_gain_per_s;
        self.planar_config.maximum_yaw_acceleration_rad_s2 = maximum_yaw_acceleration_rad_s2;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn configure_lateral_viability(
        &mut self,
        support_reserve_m: f64,
        activation_release_dcm_m: f64,
        activation_full_dcm_m: f64,
        dcm_decay_rate_per_s: f64,
        maximum_lateral_acceleration_m_s2: f64,
        lateral_acceleration_slew_m_s3: f64,
        maximum_bank_angle_rad: f64,
        bank_angle_slew_rad_s: f64,
        bank_tracking_stiffness_per_s2: f64,
        bank_tracking_damping_per_s: f64,
        maximum_roll_acceleration_rad_s2: f64,
    ) -> PyResult<()> {
        let values = [
            support_reserve_m,
            activation_release_dcm_m,
            activation_full_dcm_m,
            dcm_decay_rate_per_s,
            maximum_lateral_acceleration_m_s2,
            lateral_acceleration_slew_m_s3,
            maximum_bank_angle_rad,
            bank_angle_slew_rad_s,
            bank_tracking_stiffness_per_s2,
            bank_tracking_damping_per_s,
            maximum_roll_acceleration_rad_s2,
        ];
        if values.iter().any(|value| !value.is_finite())
            || support_reserve_m < 0.0
            || activation_release_dcm_m < 0.0
            || activation_release_dcm_m >= activation_full_dcm_m
            || dcm_decay_rate_per_s <= 0.0
            || maximum_lateral_acceleration_m_s2 <= 0.0
            || lateral_acceleration_slew_m_s3 <= 0.0
            || maximum_bank_angle_rad <= 0.0
            || maximum_bank_angle_rad >= std::f64::consts::FRAC_PI_2
            || bank_angle_slew_rad_s <= 0.0
            || bank_tracking_stiffness_per_s2 <= 0.0
            || bank_tracking_damping_per_s <= 0.0
            || maximum_roll_acceleration_rad_s2 <= 0.0
        {
            return Err(PyValueError::new_err(
                "lateral viability configuration must be finite with ordered positive bounds",
            ));
        }
        self.lateral_viability_config.support_reserve_m = support_reserve_m;
        self.lateral_viability_config.activation_release_dcm_m = activation_release_dcm_m;
        self.lateral_viability_config.activation_full_dcm_m = activation_full_dcm_m;
        self.lateral_viability_config.dcm_decay_rate_per_s = dcm_decay_rate_per_s;
        self.lateral_viability_config
            .maximum_lateral_acceleration_m_s2 = maximum_lateral_acceleration_m_s2;
        self.lateral_viability_config.lateral_acceleration_slew_m_s3 =
            lateral_acceleration_slew_m_s3;
        self.lateral_viability_config.maximum_bank_angle_rad = maximum_bank_angle_rad;
        self.lateral_viability_config.bank_angle_slew_rad_s = bank_angle_slew_rad_s;
        self.lateral_viability_config.bank_tracking_stiffness_per_s2 =
            bank_tracking_stiffness_per_s2;
        self.lateral_viability_config.bank_tracking_damping_per_s = bank_tracking_damping_per_s;
        self.lateral_viability_config
            .maximum_roll_acceleration_rad_s2 = maximum_roll_acceleration_rad_s2;
        self.lateral_viability_state = UpkieLateralViabilityState::default();
        Ok(())
    }

    #[getter]
    fn capture_diagnostic_names(&self) -> [&'static str; 16] {
        [
            "virtual_pitch_rad",
            "capture_position_control_world_m",
            "capture_error_m",
            "capture_pressure",
            "station_error_m",
            "station_authority",
            "ground_position_control_world_m",
            "ground_velocity_control_world_mps",
            "station_position_control_world_m",
            "commanded_ground_velocity_mps",
            "ground_position_odom_m",
            "ground_position_map_m",
            "center_of_mass_control_world_m",
            "center_of_mass_velocity_control_world_mps",
            "natural_frequency_rad_s",
            "used_center_of_mass_height_m",
        ]
    }

    #[getter]
    fn planar_capture_diagnostic_names(&self) -> [&'static str; 44] {
        [
            "virtual_pitch_rad",
            "capture_position_control_world_m",
            "capture_error_m",
            "capture_pressure",
            "station_error_m",
            "station_authority",
            "ground_position_control_world_m",
            "ground_velocity_control_world_mps",
            "station_position_control_world_m",
            "commanded_ground_velocity_mps",
            "ground_position_odom_m",
            "ground_position_map_m",
            "center_of_mass_control_world_m",
            "center_of_mass_velocity_control_world_mps",
            "natural_frequency_rad_s",
            "used_center_of_mass_height_m",
            "heading_world_rad",
            "longitudinal_capture_error_m",
            "lateral_capture_error_m",
            "planar_capture_error_m",
            "lateral_capture_pressure",
            "planar_capture_pressure",
            "desired_heading_error_rad",
            "target_yaw_rate_rad_s",
            "commanded_yaw_rate_rad_s",
            "commanded_yaw_acceleration_rad_s2",
            "wheel_track_m",
            "steering_was_saturated",
            "steering_direction",
            "lateral_support_margin_m",
            "lateral_dcm_m",
            "viability_support_limit_m",
            "viability_margin_m",
            "unconstrained_zmp_m",
            "commanded_zmp_m",
            "requested_lateral_acceleration_m_s2",
            "commanded_lateral_acceleration_m_s2",
            "target_bank_angle_rad",
            "commanded_bank_angle_rad",
            "commanded_roll_acceleration_rad_s2",
            "viability_activation_pressure",
            "viability_zmp_was_saturated",
            "viability_acceleration_was_saturated",
            "viability_bank_was_saturated",
        ]
    }

    #[getter]
    fn fall_safe_diagnostic_names(&self) -> [&'static str; 19] {
        [
            "mode",
            "tilt_rad",
            "angular_rate_rad_s",
            "root_height_m",
            "tilt_pressure",
            "angular_rate_pressure",
            "height_pressure",
            "solver_pressure",
            "raw_risk",
            "requested_primary_authority",
            "primary_authority",
            "contingency_authority",
            "fresh_command_authority",
            "hold_remaining_s",
            "consecutive_nonadmitted_steps",
            "transition_count",
            "limiting_reason_flags",
            "authority_was_slew_limited",
            "fallen_latched",
        ]
    }

    #[getter]
    fn support_contingency_diagnostic_names(&self) -> [&'static str; 17] {
        [
            "mode",
            "support_mask",
            "active_support_count",
            "center_of_mass_error_x_m",
            "center_of_mass_error_y_m",
            "center_of_mass_error_z_m",
            "requested_horizontal_acceleration_x_m_s2",
            "requested_horizontal_acceleration_y_m_s2",
            "requested_horizontal_acceleration_z_m_s2",
            "requested_tilt_x_rad",
            "requested_tilt_y_rad",
            "requested_tilt_z_rad",
            "ballistic_vertical_acceleration",
            "horizontal_acceleration_was_saturated",
            "vertical_acceleration_was_saturated",
            "angular_acceleration_was_saturated",
            "joint_acceleration_was_saturated",
        ]
    }

    #[getter]
    fn contact_observation_diagnostic_names(&self) -> [&'static str; 16] {
        [
            "status",
            "provenance",
            "accepted",
            "hard_constraint_eligible",
            "age_ns",
            "raw_mask",
            "debounced_mask",
            "hard_mask",
            "left_pending_samples",
            "right_pending_samples",
            "transition_count",
            "flags",
            "mapped_time_ns",
            "source_sequence",
            "source_identity",
            "synchronization_uncertainty_ns",
        ]
    }

    fn configure_contact_command_lease(&mut self, maximum_hold_ticks: u32) {
        self.contact_command_lease_config.maximum_hold_ticks = maximum_hold_ticks;
        self.contact_command_lease_state = ContactCommandLeaseState::default();
    }

    #[getter]
    fn contact_command_lease_diagnostic_names(&self) -> [&'static str; 14] {
        [
            "status",
            "provenance",
            "executable",
            "command_age_ticks",
            "remaining_hold_ticks",
            "authoring_mask",
            "stable_mask",
            "hard_mask",
            "transition_count",
            "flags",
            "tick_sequence",
            "maximum_hold_ticks",
            "observation_exact",
            "fresh_command_available",
        ]
    }

    /// Configure the generic planner-request supervisor. Request coordinates
    /// are `[root roll acceleration, root lateral acceleration, root yaw
    /// acceleration]`; Python remains responsible for planner orchestration.
    fn configure_viability_request(
        &mut self,
        activation_pressure: f64,
        release_pressure: f64,
        maximum_candidate_age_ticks: u32,
        maximum_abs_request: PyReadonlyArray1<'_, f64>,
        maximum_slew_per_tick: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        let maximum_abs_request = maximum_abs_request.as_slice()?;
        let maximum_slew_per_tick = maximum_slew_per_tick.as_slice()?;
        if maximum_abs_request.len() != 3
            || maximum_slew_per_tick.len() != 3
            || !activation_pressure.is_finite()
            || !release_pressure.is_finite()
            || release_pressure < 0.0
            || release_pressure >= activation_pressure
            || maximum_abs_request
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
            || maximum_slew_per_tick
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(PyValueError::new_err(
                "viability request configuration expects ordered nonnegative pressure thresholds and finite positive maximum_abs_request[3] and maximum_slew_per_tick[3]",
            ));
        }
        self.viability_request_config = ViabilityRequestConfig {
            activation_pressure,
            release_pressure,
            maximum_candidate_age_ticks,
            maximum_abs_request: [
                maximum_abs_request[0],
                maximum_abs_request[1],
                maximum_abs_request[2],
            ],
            maximum_slew_per_tick: [
                maximum_slew_per_tick[0],
                maximum_slew_per_tick[1],
                maximum_slew_per_tick[2],
            ],
        };
        self.viability_request_state = ViabilityRequestState::default();
        Ok(())
    }

    #[getter]
    fn viability_request_diagnostic_names(&self) -> [&'static str; 17] {
        [
            "status",
            "provenance",
            "active",
            "executable",
            "request_was_slew_limited",
            "candidate_age_ticks",
            "remaining_fresh_ticks",
            "last_pressure",
            "target_roll_acceleration",
            "target_lateral_acceleration",
            "target_yaw_acceleration",
            "transition_count",
            "flags",
            "tick_sequence",
            "planner_update",
            "observation_exact",
            "candidate_available",
        ]
    }

    /// Supervise one explicit planner tick without allocation. A failed active
    /// planner update, stale candidate, or inexact observation revokes the
    /// request. Every nonzero result still requires downstream WBC admission.
    #[allow(clippy::too_many_arguments)]
    fn step_viability_request(
        &mut self,
        tick_sequence: u64,
        observation_exact: bool,
        planner_update: bool,
        pressure: f64,
        candidate_available: bool,
        candidate: PyReadonlyArray1<'_, f64>,
        mut request_out: PyReadwriteArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<()> {
        let candidate = candidate.as_slice()?;
        let request_out = request_out.as_slice_mut()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        if candidate.len() != 3 || request_out.len() != 3 || diagnostics_out.len() != 17 {
            return Err(PyValueError::new_err(
                "viability request step expects candidate[3], request_out[3], and diagnostics_out[17]",
            ));
        }
        let allocation_before = allocation_snapshot();
        let output = step_viability_request(
            tick_sequence,
            observation_exact,
            planner_update,
            pressure,
            candidate_available,
            [candidate[0], candidate[1], candidate[2]],
            self.viability_request_config,
            &mut self.viability_request_state,
        );
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "viability request supervision allocated inside the Rust hot path",
            ));
        }
        request_out.copy_from_slice(&output.request);
        diagnostics_out.copy_from_slice(&[
            output.status as u8 as f64,
            output.provenance as u8 as f64,
            f64::from(output.active),
            f64::from(output.executable),
            f64::from(output.request_was_slew_limited),
            output.candidate_age_ticks as f64,
            f64::from(output.remaining_fresh_ticks),
            output.last_pressure,
            output.target[0],
            output.target[1],
            output.target[2],
            f64::from(output.transition_count),
            f64::from(output.flags),
            tick_sequence as f64,
            f64::from(planner_update),
            f64::from(observation_exact),
            f64::from(candidate_available),
        ]);
        Ok(())
    }

    /// Configure the fail-closed shadow confirmation between a reduced-model
    /// planner and the ordinary viability-request supervisor.
    fn configure_viability_confirmation(
        &mut self,
        minimum_consistent_updates: u32,
        maximum_update_gap_ticks: u32,
        minimum_score_improvement: f64,
        minimum_normalized_alignment: f64,
        maximum_abs_candidate: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        let maximum = maximum_abs_candidate.as_slice()?;
        if minimum_consistent_updates == 0
            || !minimum_score_improvement.is_finite()
            || minimum_score_improvement < 0.0
            || !minimum_normalized_alignment.is_finite()
            || !(-1.0..=1.0).contains(&minimum_normalized_alignment)
            || maximum.len() != 3
            || maximum
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(PyValueError::new_err(
                "viability confirmation expects a positive update count, finite nonnegative improvement, alignment in [-1, 1], and finite positive maximum_abs_candidate[3]",
            ));
        }
        self.viability_confirmation_config = ViabilityConfirmationConfig {
            minimum_consistent_updates,
            maximum_update_gap_ticks,
            minimum_score_improvement,
            minimum_normalized_alignment,
            maximum_abs_candidate: [maximum[0], maximum[1], maximum[2]],
        };
        self.viability_confirmation_state = ViabilityConfirmationState::default();
        Ok(())
    }

    #[getter]
    fn viability_confirmation_diagnostic_names(&self) -> [&'static str; 22] {
        [
            "status",
            "executable",
            "has_shadow",
            "support_mask",
            "consistent_update_count",
            "shadow_age_ticks",
            "normalized_alignment",
            "score_improvement",
            "baseline_score",
            "candidate_score",
            "shadow_roll_acceleration",
            "shadow_lateral_acceleration",
            "shadow_yaw_acceleration",
            "confirmed_roll_acceleration",
            "confirmed_lateral_acceleration",
            "confirmed_yaw_acceleration",
            "transition_count",
            "flags",
            "tick_sequence",
            "planner_update",
            "observation_exact",
            "candidate_available",
        ]
    }

    /// Confirm or revoke one planner proposal. All arrays are caller-owned and
    /// the timed Rust state transition must remain allocation-free.
    #[allow(clippy::too_many_arguments)]
    fn step_viability_confirmation(
        &mut self,
        tick_sequence: u64,
        observation_exact: bool,
        planner_update: bool,
        support_mask: u32,
        candidate_available: bool,
        baseline_score: f64,
        candidate_score: f64,
        candidate: PyReadonlyArray1<'_, f64>,
        mut confirmed_out: PyReadwriteArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let candidate = candidate.as_slice()?;
        let confirmed = confirmed_out.as_slice_mut()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        if candidate.len() != 3 || confirmed.len() != 3 || diagnostics.len() != 22 {
            return Err(PyValueError::new_err(
                "viability confirmation expects candidate[3], confirmed_out[3], and diagnostics_out[22]",
            ));
        }
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let output = step_viability_confirmation(
            tick_sequence,
            observation_exact,
            planner_update,
            support_mask,
            candidate_available,
            baseline_score,
            candidate_score,
            [candidate[0], candidate[1], candidate[2]],
            self.viability_confirmation_config,
            &mut self.viability_confirmation_state,
        );
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        confirmed.copy_from_slice(&output.confirmed_candidate);
        diagnostics.copy_from_slice(&[
            output.status as u8 as f64,
            f64::from(output.executable),
            f64::from(output.has_shadow),
            output.support_mask as f64,
            output.consistent_update_count as f64,
            output.shadow_age_ticks as f64,
            output.normalized_alignment,
            output.score_improvement,
            output.baseline_score,
            output.candidate_score,
            output.shadow_candidate[0],
            output.shadow_candidate[1],
            output.shadow_candidate[2],
            output.confirmed_candidate[0],
            output.confirmed_candidate[1],
            output.confirmed_candidate[2],
            output.transition_count as f64,
            output.flags as f64,
            tick_sequence as f64,
            f64::from(planner_update),
            f64::from(observation_exact),
            f64::from(candidate_available),
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    fn configure_viability_hybrid_guard(
        &mut self,
        minimum_support_age_ticks: u32,
        minimum_double_support_load_fraction: f64,
        minimum_opening_roll_capture_pressure: f64,
        maximum_abs_request: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        let maximum = maximum_abs_request.as_slice()?;
        if minimum_support_age_ticks == 0
            || !minimum_double_support_load_fraction.is_finite()
            || !(0.0..0.5).contains(&minimum_double_support_load_fraction)
            || !minimum_opening_roll_capture_pressure.is_finite()
            || minimum_opening_roll_capture_pressure <= 0.0
            || maximum.len() != 3
            || maximum
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(PyValueError::new_err(
                "hybrid guard expects positive support age, double-support load fraction in [0, 0.5), positive opening roll-capture pressure, and finite positive maximum_abs_request[3]",
            ));
        }
        self.viability_hybrid_guard_config = ViabilityHybridGuardConfig {
            minimum_support_age_ticks,
            minimum_double_support_load_fraction,
            minimum_opening_roll_capture_pressure,
            maximum_abs_request: [maximum[0], maximum[1], maximum[2]],
        };
        self.viability_hybrid_guard_state = ViabilityHybridGuardState::default();
        Ok(())
    }

    #[getter]
    fn viability_hybrid_guard_diagnostic_names(&self) -> [&'static str; 13] {
        [
            "status",
            "executable",
            "shadow_admissible",
            "support_mask",
            "support_age_ticks",
            "minimum_load_fraction",
            "transition_count",
            "flags",
            "tick_sequence",
            "observation_exact",
            "candidate_available",
            "signed_roll_capture_pressure",
            "request_roll_acceleration",
        ]
    }

    #[allow(clippy::too_many_arguments)]
    fn step_viability_hybrid_guard(
        &mut self,
        tick_sequence: u64,
        observation_exact: bool,
        candidate_available: bool,
        support_mask: u8,
        signed_roll_capture_pressure: f64,
        request: PyReadonlyArray1<'_, f64>,
        candidate_normal_force: PyReadonlyArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let request = request.as_slice()?;
        let force = candidate_normal_force.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        if request.len() != 3 || force.len() != 2 || diagnostics.len() != 13 {
            return Err(PyValueError::new_err(
                "hybrid guard expects request[3], candidate_normal_force[2], and diagnostics_out[13]",
            ));
        }
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let output = step_viability_hybrid_guard(
            tick_sequence,
            observation_exact,
            candidate_available,
            support_mask,
            signed_roll_capture_pressure,
            [request[0], request[1], request[2]],
            [force[0], force[1]],
            self.viability_hybrid_guard_config,
            &mut self.viability_hybrid_guard_state,
        );
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        diagnostics.copy_from_slice(&[
            output.status as u8 as f64,
            f64::from(output.executable),
            f64::from(output.shadow_admissible),
            output.support_mask as f64,
            output.support_age_ticks as f64,
            output.minimum_load_fraction,
            output.transition_count as f64,
            output.flags as f64,
            tick_sequence as f64,
            f64::from(observation_exact),
            f64::from(candidate_available),
            signed_roll_capture_pressure,
            request[0],
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    fn configure_viability_execution_monitor(
        &mut self,
        minimum_samples: u32,
        reserve_multiplier: f64,
        minimum_normalized_bound: PyReadonlyArray1<'_, f64>,
        maximum_normalized_bound: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        let minimum = minimum_normalized_bound.as_slice()?;
        let maximum = maximum_normalized_bound.as_slice()?;
        if minimum_samples == 0
            || minimum_samples > 32
            || !reserve_multiplier.is_finite()
            || reserve_multiplier < 1.0
            || minimum.len() != VIABILITY_EXECUTION_COMPONENTS
            || maximum.len() != VIABILITY_EXECUTION_COMPONENTS
            || minimum.iter().zip(maximum).any(|(low, high)| {
                !low.is_finite() || *low < 0.0 || !high.is_finite() || *high <= 0.0 || low > high
            })
        {
            return Err(PyValueError::new_err(
                "execution monitor expects minimum_samples in [1, 32], reserve >= 1, and finite minimum/maximum bound arrays with 0 <= minimum <= maximum",
            ));
        }
        let mut minimum_bound = [0.0; VIABILITY_EXECUTION_COMPONENTS];
        let mut maximum_bound = [0.0; VIABILITY_EXECUTION_COMPONENTS];
        minimum_bound.copy_from_slice(minimum);
        maximum_bound.copy_from_slice(maximum);
        self.viability_execution_monitor_config = ViabilityExecutionMonitorConfig {
            minimum_samples,
            reserve_multiplier,
            minimum_normalized_bound: minimum_bound,
            maximum_normalized_bound: maximum_bound,
        };
        self.viability_execution_monitor_state = ViabilityExecutionMonitorState::default();
        Ok(())
    }

    #[getter]
    fn viability_execution_monitor_diagnostic_names(&self) -> [&'static str; 26] {
        [
            "status",
            "certificate_valid",
            "support_mask",
            "sample_count",
            "maximum_error_ratio",
            "transition_count",
            "flags",
            "tick_sequence",
            "observation_exact",
            "prediction_available",
            "roll_error",
            "roll_rate_error",
            "pitch_error",
            "pitch_rate_error",
            "lateral_error",
            "lateral_rate_error",
            "yaw_error",
            "yaw_rate_error",
            "roll_bound",
            "roll_rate_bound",
            "pitch_bound",
            "pitch_rate_bound",
            "lateral_bound",
            "lateral_rate_bound",
            "yaw_bound",
            "yaw_rate_bound",
        ]
    }

    #[allow(clippy::too_many_arguments)]
    fn step_viability_execution_monitor(
        &mut self,
        tick_sequence: u64,
        observation_exact: bool,
        prediction_available: bool,
        support_mask: u8,
        predicted_state: PyReadonlyArray1<'_, f64>,
        observed_state: PyReadonlyArray1<'_, f64>,
        normalization_scale: PyReadonlyArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let predicted = predicted_state.as_slice()?;
        let observed = observed_state.as_slice()?;
        let scale = normalization_scale.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        if predicted.len() != VIABILITY_EXECUTION_COMPONENTS
            || observed.len() != VIABILITY_EXECUTION_COMPONENTS
            || scale.len() != VIABILITY_EXECUTION_COMPONENTS
            || diagnostics.len() != 26
        {
            return Err(PyValueError::new_err(
                "execution monitor expects predicted/observed/scale[8] and diagnostics_out[26]",
            ));
        }
        let mut predicted_fixed = [0.0; VIABILITY_EXECUTION_COMPONENTS];
        let mut observed_fixed = [0.0; VIABILITY_EXECUTION_COMPONENTS];
        let mut scale_fixed = [0.0; VIABILITY_EXECUTION_COMPONENTS];
        predicted_fixed.copy_from_slice(predicted);
        observed_fixed.copy_from_slice(observed);
        scale_fixed.copy_from_slice(scale);
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let output = step_viability_execution_monitor(
            tick_sequence,
            observation_exact,
            prediction_available,
            support_mask,
            predicted_fixed,
            observed_fixed,
            scale_fixed,
            self.viability_execution_monitor_config,
            &mut self.viability_execution_monitor_state,
        );
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        diagnostics[..10].copy_from_slice(&[
            output.status as u8 as f64,
            f64::from(output.certificate_valid),
            output.support_mask as f64,
            output.sample_count as f64,
            output.maximum_error_ratio,
            output.transition_count as f64,
            output.flags as f64,
            tick_sequence as f64,
            f64::from(observation_exact),
            f64::from(prediction_available),
        ]);
        diagnostics[10..18].copy_from_slice(&output.normalized_error);
        diagnostics[18..26].copy_from_slice(&output.normalized_bound);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    #[getter]
    fn viability_poll_diagnostic_names(&self) -> [&'static str; 4] {
        ["axis", "direction", "phase", "saturated"]
    }

    /// Advance one fixed-size Rust-owned signed-coordinate proposal. The
    /// planner still has to query the exact WBC and compare the proposal with
    /// a same-state zero baseline before it can become a supervised request.
    fn next_viability_poll(
        &mut self,
        previous_request: PyReadonlyArray1<'_, f64>,
        maximum_abs_request: PyReadonlyArray1<'_, f64>,
        local_step: PyReadonlyArray1<'_, f64>,
        mut proposal_out: PyReadwriteArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let previous = previous_request.as_slice()?;
        let maximum = maximum_abs_request.as_slice()?;
        let step = local_step.as_slice()?;
        let proposal = proposal_out.as_slice_mut()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        if previous.len() != 3
            || maximum.len() != 3
            || step.len() != 3
            || proposal.len() != 3
            || diagnostics.len() != 4
        {
            return Err(PyValueError::new_err(
                "viability poll expects previous/max/step/proposal[3] and diagnostics[4]",
            ));
        }
        let config = ViabilityPollConfig {
            maximum_abs_request: [maximum[0], maximum[1], maximum[2]],
            local_step: [step[0], step[1], step[2]],
        };
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let output = next_viability_poll(
            [previous[0], previous[1], previous[2]],
            config,
            &mut self.viability_poll_state,
        )
        .map_err(|error| PyValueError::new_err(format!("invalid viability poll: {error:?}")))?;
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        proposal.copy_from_slice(&output.proposal);
        diagnostics.copy_from_slice(&[
            output.axis as f64,
            output.direction as f64,
            output.phase as f64,
            f64::from(output.saturated),
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Emit the eight reduced-order state knots used by the forecast scorer.
    /// Path columns are `[time, roll, roll_rate, pitch, pitch_rate, lateral,
    /// lateral_velocity, yaw, yaw_rate]`.
    #[allow(clippy::too_many_arguments)]
    fn predict_viability_forecast_path(
        &self,
        state: PyReadonlyArray1<'_, f64>,
        achieved_acceleration: PyReadonlyArray1<'_, f64>,
        request: PyReadonlyArray1<'_, f64>,
        previous_request: PyReadonlyArray1<'_, f64>,
        maximum_abs_request: PyReadonlyArray1<'_, f64>,
        maximum_torque_utilization: f64,
        minimum_joint_headroom_fraction: f64,
        mut path_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let path_shape = path_out.as_array().dim();
        let state = state.as_slice()?;
        let achieved = achieved_acceleration.as_slice()?;
        let request = request.as_slice()?;
        let previous = previous_request.as_slice()?;
        let maximum = maximum_abs_request.as_slice()?;
        let path = path_out.as_slice_mut()?;
        if state.len() != 10
            || achieved.len() != 4
            || request.len() != 3
            || previous.len() != 3
            || maximum.len() != 3
            || path_shape != (VIABILITY_FORECAST_KNOTS, 9)
            || !state[9].is_finite()
            || state[9] < 0.0
            || state[9] > 3.0
            || state[9].fract() != 0.0
        {
            return Err(PyValueError::new_err(
                "viability path expects state[10], achieved[4], request/previous/max[3], and path_out[8,9]",
            ));
        }
        let forecast_state = ViabilityForecastState {
            roll_rad: state[0],
            roll_rate_rad_s: state[1],
            pitch_rad: state[2],
            pitch_rate_rad_s: state[3],
            lateral_position_m: state[4],
            lateral_velocity_m_s: state[5],
            yaw_rad: state[6],
            yaw_rate_rad_s: state[7],
            center_of_mass_height_m: state[8],
            support_mask: state[9] as u8,
        };
        let candidate = ViabilityForecastCandidate {
            achieved_acceleration: [achieved[0], achieved[1], achieved[2], achieved[3]],
            request: [request[0], request[1], request[2]],
            previous_request: [previous[0], previous[1], previous[2]],
            maximum_abs_request: [maximum[0], maximum[1], maximum[2]],
            maximum_torque_utilization,
            minimum_joint_headroom_fraction,
        };
        let mut knots = [ViabilityForecastKnot::default(); VIABILITY_FORECAST_KNOTS];
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        predict_viability_forecast_path(
            forecast_state,
            candidate,
            self.viability_forecast_config,
            &mut knots,
        )
        .map_err(|error| PyValueError::new_err(format!("invalid viability path: {error:?}")))?;
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        for (row, knot) in knots.iter().enumerate() {
            path[row * 9..(row + 1) * 9].copy_from_slice(&[
                knot.time_s,
                knot.roll_rad,
                knot.roll_rate_rad_s,
                knot.pitch_rad,
                knot.pitch_rate_rad_s,
                knot.lateral_position_m,
                knot.lateral_velocity_m_s,
                knot.yaw_rad,
                knot.yaw_rate_rad_s,
            ]);
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    #[getter]
    fn viability_forecast_diagnostic_names(&self) -> [&'static str; 15] {
        [
            "total_score",
            "activation_pressure",
            "current_capture_pressure",
            "current_sagittal_pressure",
            "peak_capture_pressure",
            "peak_sagittal_pressure",
            "terminal_capture_pressure",
            "terminal_sagittal_pressure",
            "terminal_rate_pressure",
            "peak_yaw_pressure",
            "resource_pressure",
            "action_pressure",
            "action_delta_pressure",
            "support_pressure",
            "minimum_capture_margin",
        ]
    }

    /// Score a caller-owned batch of exact-WBC candidates at eight fixed
    /// future knots. State layout is `[roll, roll_rate, pitch, pitch_rate,
    /// lateral_position, lateral_velocity, yaw, yaw_rate, CoM_height,
    /// support_mask]`; achieved acceleration is `[roll, lateral, yaw, pitch]`
    /// and request layout is `[roll, lateral, yaw]`.
    #[allow(clippy::too_many_arguments)]
    fn score_viability_forecast_batch(
        &self,
        state: PyReadonlyArray1<'_, f64>,
        achieved_accelerations: PyReadonlyArray2<'_, f64>,
        requests: PyReadonlyArray2<'_, f64>,
        previous_request: PyReadonlyArray1<'_, f64>,
        maximum_abs_request: PyReadonlyArray1<'_, f64>,
        maximum_torque_utilization: PyReadonlyArray1<'_, f64>,
        minimum_joint_headroom_fraction: PyReadonlyArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let achieved_shape = achieved_accelerations.as_array().dim();
        let request_shape = requests.as_array().dim();
        let diagnostics_shape = diagnostics_out.as_array().dim();
        let state = state.as_slice()?;
        let achieved = achieved_accelerations.as_slice()?;
        let requests = requests.as_slice()?;
        let previous = previous_request.as_slice()?;
        let maximum = maximum_abs_request.as_slice()?;
        let torque = maximum_torque_utilization.as_slice()?;
        let joint = minimum_joint_headroom_fraction.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let rows = torque.len();
        if state.len() != 10
            || previous.len() != 3
            || maximum.len() != 3
            || joint.len() != rows
            || achieved.len() != rows * 4
            || requests.len() != rows * 3
            || diagnostics.len() != rows * 15
            || achieved_shape != (rows, 4)
            || request_shape != (rows, 3)
            || diagnostics_shape != (rows, 15)
            || !state[9].is_finite()
            || state[9] < 0.0
            || state[9] > 3.0
            || state[9].fract() != 0.0
        {
            return Err(PyValueError::new_err(
                "viability forecast expects state[10], achieved[N,4], request[N,3], evidence[N], previous/max[3], and diagnostics[N,15]",
            ));
        }
        let forecast_state = ViabilityForecastState {
            roll_rad: state[0],
            roll_rate_rad_s: state[1],
            pitch_rad: state[2],
            pitch_rate_rad_s: state[3],
            lateral_position_m: state[4],
            lateral_velocity_m_s: state[5],
            yaw_rad: state[6],
            yaw_rate_rad_s: state[7],
            center_of_mass_height_m: state[8],
            support_mask: state[9] as u8,
        };
        let candidate_at = |row: usize| ViabilityForecastCandidate {
            achieved_acceleration: [
                achieved[row * 4],
                achieved[row * 4 + 1],
                achieved[row * 4 + 2],
                achieved[row * 4 + 3],
            ],
            request: [
                requests[row * 3],
                requests[row * 3 + 1],
                requests[row * 3 + 2],
            ],
            previous_request: [previous[0], previous[1], previous[2]],
            maximum_abs_request: [maximum[0], maximum[1], maximum[2]],
            maximum_torque_utilization: torque[row],
            minimum_joint_headroom_fraction: joint[row],
        };
        // Validate the full batch before mutating caller output.
        for row in 0..rows {
            score_viability_forecast(
                forecast_state,
                candidate_at(row),
                self.viability_forecast_config,
            )
            .map_err(|error| {
                PyValueError::new_err(format!("invalid viability forecast row {row}: {error:?}"))
            })?;
        }

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for row in 0..rows {
            // The validation pass above proves this cannot fail.
            let score = score_viability_forecast(
                forecast_state,
                candidate_at(row),
                self.viability_forecast_config,
            )
            .expect("validated viability forecast candidate");
            diagnostics[row * 15..(row + 1) * 15].copy_from_slice(&[
                score.total,
                score.activation_pressure,
                score.current_capture_pressure,
                score.current_sagittal_pressure,
                score.peak_capture_pressure,
                score.peak_sagittal_pressure,
                score.terminal_capture_pressure,
                score.terminal_sagittal_pressure,
                score.terminal_rate_pressure,
                score.peak_yaw_pressure,
                score.resource_pressure,
                score.action_pressure,
                score.action_delta_pressure,
                score.support_pressure,
                score.minimum_capture_margin,
            ]);
        }
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    #[getter]
    fn inexact_observation_authority_selector_diagnostic_names(&self) -> [&'static str; 12] {
        [
            "queried",
            "selected_index",
            "selected_authority_q15",
            "selected_score",
            "zero_authority_score",
            "full_authority_score",
            "score_improvement_from_zero",
            "score_a000",
            "score_a025",
            "score_a050",
            "score_a075",
            "score_a100",
        ]
    }

    /// Select and install the next inexact-observation retained-effort
    /// authority from a support-free reduced-order forecast. The caller owns
    /// the current state and the last admitted command witness; Rust owns the
    /// fixed candidate set, scoring, deterministic tie break, and authority
    /// update.
    fn select_contact_program_inexact_observation_authority(
        &mut self,
        state: PyReadonlyArray1<'_, f64>,
        full_authority_achieved_acceleration: PyReadonlyArray1<'_, f64>,
        full_authority_maximum_torque_utilization: f64,
        minimum_joint_headroom_fraction: f64,
        minimum_score_improvement_from_zero: f64,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let state = state.as_slice()?;
        let achieved = full_authority_achieved_acceleration.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        if state.len() != 9 || achieved.len() != 4 || diagnostics.len() != 12 {
            return Err(PyValueError::new_err(
                "inexact-observation authority selector expects state[9], achieved[4], and diagnostics[12]",
            ));
        }
        let forecast_state = ViabilityForecastState {
            roll_rad: state[0],
            roll_rate_rad_s: state[1],
            pitch_rad: state[2],
            pitch_rate_rad_s: state[3],
            lateral_position_m: state[4],
            lateral_velocity_m_s: state[5],
            yaw_rad: state[6],
            yaw_rate_rad_s: state[7],
            center_of_mass_height_m: state[8],
            support_mask: 0,
        };
        let achieved = [achieved[0], achieved[1], achieved[2], achieved[3]];
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let selected = select_inexact_observation_authority(
            forecast_state,
            achieved,
            full_authority_maximum_torque_utilization,
            minimum_joint_headroom_fraction,
            minimum_score_improvement_from_zero,
            self.viability_forecast_config,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid inexact-observation authority selector input: {error:?}"
            ))
        })?;
        self.contact_program_authority_config
            .inexact_observation_hold_authority_q15 = selected.authority_q15;
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "inexact-observation authority selector allocated inside the Rust hot path",
            ));
        }
        diagnostics.copy_from_slice(&[
            1.0,
            f64::from(selected.selected_index),
            f64::from(selected.authority_q15),
            selected.selected_score,
            selected.zero_authority_score,
            selected.full_authority_score,
            selected.score_improvement_from_zero,
            selected.candidate_scores[0],
            selected.candidate_scores[1],
            selected.candidate_scores[2],
            selected.candidate_scores[3],
            selected.candidate_scores[4],
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Derive Upkie wheel-point `M^-1 J^T` from Bonesaw's own floating model.
    /// Contact points are world-frame rows `[left, right]`; bases are
    /// `[contact, tangent_x/tangent_y/normal, world_xyz]`. Outputs are
    /// response `[D,2,3]` and directional effective mass `[2,3]`, with
    /// `D = 6 + joints`. Returned timing is
    /// `(elapsed_ns, allocation_calls, allocated_bytes)`.
    #[allow(clippy::too_many_arguments)]
    fn model_contact_impulse_velocity_response(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        contact_points_world: PyReadonlyArray2<'_, f64>,
        contact_bases_world: PyReadonlyArray3<'_, f64>,
        mut response_out: PyReadwriteArray3<'_, f64>,
        mut effective_mass_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let point_shape = contact_points_world.as_array().dim();
        let basis_shape = contact_bases_world.as_array().dim();
        let response_shape = response_out.as_array().dim();
        let effective_mass_shape = effective_mass_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let contact_points_world = contact_points_world.as_slice()?;
        let contact_bases_world = contact_bases_world.as_slice()?;
        let response_out = response_out.as_slice_mut()?;
        let effective_mass_out = effective_mass_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || point_shape != (2, 3)
            || basis_shape != (2, 3, 3)
            || response_shape != (generalized_dof, 2, 3)
            || effective_mass_shape != (2, 3)
        {
            return Err(PyValueError::new_err(format!(
                "model contact response expects root[3], quaternion[4], q[{dof}], points[2,3], bases[2,3,3], response[{generalized_dof},2,3], and effective_mass[2,3]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(contact_points_world)
            .chain(contact_bases_world)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "model contact response inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("model contact quaternion is degenerate"))?;
        let contacts: [PointImpulseResponseSpec; 2] =
            std::array::from_fn(|contact| PointImpulseResponseSpec {
                frame: self.support_frames[contact],
                point_world: Vec3::new(
                    contact_points_world[contact * 3],
                    contact_points_world[contact * 3 + 1],
                    contact_points_world[contact * 3 + 2],
                ),
                basis_world: std::array::from_fn(|axis| {
                    let start = contact * 9 + axis * 3;
                    Vec3::new(
                        contact_bases_world[start],
                        contact_bases_world[start + 1],
                        contact_bases_world[start + 2],
                    )
                }),
            });
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_point_impulse_velocity_response(
            &self.program.model,
            &self.robot,
            &contacts,
            &mut self.contact_transition_response_scratch,
            response_out,
            effective_mass_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid model contact response: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_point_impulse_velocity_response(
            &self.program.model,
            &self.robot,
            &contacts,
            &mut self.contact_transition_response_scratch,
            response_out,
            effective_mass_out,
        )
        .expect("validated model contact response");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "model contact response allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Derive the Upkie wheel-point response plus the complete coupled
    /// Delassus matrix `J M^-1 J^T`. `delassus_out` is `[6,6]` in flattened
    /// left/right tangent-X, tangent-Y, normal order.
    #[allow(clippy::too_many_arguments)]
    fn model_contact_impulse_velocity_response_with_delassus(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        contact_points_world: PyReadonlyArray2<'_, f64>,
        contact_bases_world: PyReadonlyArray3<'_, f64>,
        mut response_out: PyReadwriteArray3<'_, f64>,
        mut effective_mass_out: PyReadwriteArray2<'_, f64>,
        mut delassus_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let point_shape = contact_points_world.as_array().dim();
        let basis_shape = contact_bases_world.as_array().dim();
        let response_shape = response_out.as_array().dim();
        let effective_mass_shape = effective_mass_out.as_array().dim();
        let delassus_shape = delassus_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let contact_points_world = contact_points_world.as_slice()?;
        let contact_bases_world = contact_bases_world.as_slice()?;
        let response_out = response_out.as_slice_mut()?;
        let effective_mass_out = effective_mass_out.as_slice_mut()?;
        let delassus_out = delassus_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || point_shape != (2, 3)
            || basis_shape != (2, 3, 3)
            || response_shape != (generalized_dof, 2, 3)
            || effective_mass_shape != (2, 3)
            || delassus_shape != (6, 6)
        {
            return Err(PyValueError::new_err(format!(
                "coupled model response expects root[3], quaternion[4], q[{dof}], points[2,3], bases[2,3,3], response[{generalized_dof},2,3], effective_mass[2,3], and delassus[6,6]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(contact_points_world)
            .chain(contact_bases_world)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "coupled model response inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("model contact quaternion is degenerate"))?;
        let contacts: [PointImpulseResponseSpec; 2] =
            std::array::from_fn(|contact| PointImpulseResponseSpec {
                frame: self.support_frames[contact],
                point_world: Vec3::new(
                    contact_points_world[contact * 3],
                    contact_points_world[contact * 3 + 1],
                    contact_points_world[contact * 3 + 2],
                ),
                basis_world: std::array::from_fn(|axis| {
                    let start = contact * 9 + axis * 3;
                    Vec3::new(
                        contact_bases_world[start],
                        contact_bases_world[start + 1],
                        contact_bases_world[start + 2],
                    )
                }),
            });
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_point_impulse_velocity_response_with_delassus(
            &self.program.model,
            &self.robot,
            &contacts,
            &mut self.contact_transition_response_scratch,
            response_out,
            effective_mass_out,
            delassus_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid coupled model response: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_point_impulse_velocity_response_with_delassus(
            &self.program.model,
            &self.robot,
            &contacts,
            &mut self.contact_transition_response_scratch,
            response_out,
            effective_mass_out,
            delassus_out,
        )
        .expect("validated coupled model response");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "coupled model response allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Derive the two-wheel spatial-wrench response and complete coupled
    /// Delassus matrix. Wrench axes are moment XYZ then force XYZ about each
    /// declared reference point. This is a response query, not online contact
    /// evidence or authority.
    #[allow(clippy::too_many_arguments)]
    fn model_contact_spatial_impulse_velocity_response(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        reference_points_world: PyReadonlyArray2<'_, f64>,
        wrench_bases_world: PyReadonlyArray3<'_, f64>,
        mut response_out: PyReadwriteArray3<'_, f64>,
        mut delassus_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let point_shape = reference_points_world.as_array().dim();
        let basis_shape = wrench_bases_world.as_array().dim();
        let response_shape = response_out.as_array().dim();
        let delassus_shape = delassus_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let reference_points_world = reference_points_world.as_slice()?;
        let wrench_bases_world = wrench_bases_world.as_slice()?;
        let response_out = response_out.as_slice_mut()?;
        let delassus_out = delassus_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        let spatial_axes = 2 * SPATIAL_IMPULSE_WIDTH;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || point_shape != (2, 3)
            || basis_shape != (2, 3, 3)
            || response_shape != (generalized_dof, 2, SPATIAL_IMPULSE_WIDTH)
            || delassus_shape != (spatial_axes, spatial_axes)
        {
            return Err(PyValueError::new_err(format!(
                "spatial model response expects root[3], quaternion[4], q[{dof}], reference points[2,3], bases[2,3,3], response[{generalized_dof},2,6], and delassus[12,12]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(reference_points_world)
            .chain(wrench_bases_world)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "spatial model response inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("spatial model response quaternion is degenerate"))?;
        let wrenches: [SpatialImpulseResponseSpec; 2] =
            std::array::from_fn(|wrench| SpatialImpulseResponseSpec {
                frame: self.support_frames[wrench],
                reference_point_world: Vec3::new(
                    reference_points_world[wrench * 3],
                    reference_points_world[wrench * 3 + 1],
                    reference_points_world[wrench * 3 + 2],
                ),
                basis_world: std::array::from_fn(|axis| {
                    let start = wrench * 9 + axis * 3;
                    Vec3::new(
                        wrench_bases_world[start],
                        wrench_bases_world[start + 1],
                        wrench_bases_world[start + 2],
                    )
                }),
            });
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_spatial_impulse_velocity_response(
            &self.program.model,
            &self.robot,
            &wrenches,
            &mut self.contact_transition_response_scratch,
            response_out,
            delassus_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid spatial model response: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_spatial_impulse_velocity_response(
            &self.program.model,
            &self.robot,
            &wrenches,
            &mut self.contact_transition_response_scratch,
            response_out,
            delassus_out,
        )
        .expect("validated spatial model response");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "spatial model response allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Compute `M(q) * (observed_delta - predicted_delta)` for each candidate.
    /// This diagnostic remains separate from contact-response authority.
    #[allow(clippy::too_many_arguments)]
    fn model_generalized_momentum_impulse_residuals(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        observed_delta_velocity: PyReadonlyArray1<'_, f64>,
        predicted_delta_velocity: PyReadonlyArray2<'_, f64>,
        mut residual_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let predicted_shape = predicted_delta_velocity.as_array().dim();
        let residual_shape = residual_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let observed_delta_velocity = observed_delta_velocity.as_slice()?;
        let predicted_delta_velocity = predicted_delta_velocity.as_slice()?;
        let residual_out = residual_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || observed_delta_velocity.len() != generalized_dof
            || predicted_shape.0 == 0
            || predicted_shape.1 != generalized_dof
            || residual_shape != predicted_shape
        {
            return Err(PyValueError::new_err(format!(
                "generalized momentum residual expects root[3], quaternion[4], q[{dof}], observed[{generalized_dof}], predicted[C,{generalized_dof}], and residual[C,{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(observed_delta_velocity)
            .chain(predicted_delta_velocity)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "generalized momentum residual inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| {
            PyValueError::new_err("generalized momentum residual quaternion is degenerate")
        })?;
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_momentum_impulse_residuals(
            &self.program.model,
            &self.robot,
            observed_delta_velocity,
            predicted_delta_velocity,
            &mut self.contact_transition_response_scratch,
            residual_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid generalized momentum residual: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_momentum_impulse_residuals(
            &self.program.model,
            &self.robot,
            observed_delta_velocity,
            predicted_delta_velocity,
            &mut self.contact_transition_response_scratch,
            residual_out,
        )
        .expect("validated generalized momentum residual");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "generalized momentum residual allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Map a generalized-momentum impulse box through the exact model inverse
    /// mass. Momentum and velocity use `[root angular; root linear; joints]`.
    #[allow(clippy::too_many_arguments)]
    fn model_generalized_velocity_interval_from_momentum_box(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        momentum_lower: PyReadonlyArray1<'_, f64>,
        momentum_upper: PyReadonlyArray1<'_, f64>,
        mut velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let momentum_lower = momentum_lower.as_slice()?;
        let momentum_upper = momentum_upper.as_slice()?;
        let velocity_lower_out = velocity_lower_out.as_slice_mut()?;
        let velocity_upper_out = velocity_upper_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || momentum_lower.len() != generalized_dof
            || momentum_upper.len() != generalized_dof
            || velocity_lower_out.len() != generalized_dof
            || velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "momentum box expects root[3], quaternion[4], q[{dof}], momentum lower/upper[{generalized_dof}], and velocity lower/upper[{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "momentum-box model state must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("momentum-box quaternion is degenerate"))?;
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_velocity_interval_from_momentum_box(
            &self.program.model,
            &self.robot,
            momentum_lower,
            momentum_upper,
            &mut self.contact_transition_response_scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .map_err(|error| PyValueError::new_err(format!("invalid momentum box: {error:?}")))?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_velocity_interval_from_momentum_box(
            &self.program.model,
            &self.robot,
            momentum_lower,
            momentum_upper,
            &mut self.contact_transition_response_scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .expect("validated momentum box");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "momentum-box projection allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Bound a candidate's generalized velocity jump across an uncertain
    /// contact transition. Contact witnesses are rows
    /// `[closing_speed, effective_normal_mass, sustained_normal_force, mu]`;
    /// the response is caller-owned `M^-1 J^T` with shape `[D,C,3]`.
    /// Returned timing is `(elapsed_ns, allocation_calls, allocated_bytes)`.
    #[allow(clippy::too_many_arguments)]
    fn bound_contact_transition_velocity_jump(
        &self,
        transition_time_s: PyReadonlyArray1<'_, f64>,
        restitution_upper: f64,
        contact_witnesses: PyReadonlyArray2<'_, f64>,
        generalized_acceleration: PyReadonlyArray1<'_, f64>,
        impulse_velocity_response: PyReadonlyArray3<'_, f64>,
        mut impulse_upper_out: PyReadwriteArray2<'_, f64>,
        mut delta_velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut delta_velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let witness_shape = contact_witnesses.as_array().dim();
        let response_shape = impulse_velocity_response.as_array().dim();
        let impulse_shape = impulse_upper_out.as_array().dim();
        let transition_time_s = transition_time_s.as_slice()?;
        let contact_witnesses = contact_witnesses.as_slice()?;
        let generalized_acceleration = generalized_acceleration.as_slice()?;
        let impulse_velocity_response = impulse_velocity_response.as_slice()?;
        let impulse_upper_out = impulse_upper_out.as_slice_mut()?;
        let delta_velocity_lower_out = delta_velocity_lower_out.as_slice_mut()?;
        let delta_velocity_upper_out = delta_velocity_upper_out.as_slice_mut()?;
        let dof = generalized_acceleration.len();
        let contacts = witness_shape.0;
        if transition_time_s.len() != 2
            || witness_shape.1 != CONTACT_TRANSITION_WITNESS_WIDTH
            || response_shape != (dof, contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || impulse_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || delta_velocity_lower_out.len() != dof
            || delta_velocity_upper_out.len() != dof
        {
            return Err(PyValueError::new_err(
                "contact transition expects time[2], witnesses[C,4], acceleration[D], response[D,C,3], impulse_upper[C,3], and lower/upper[D]",
            ));
        }
        let input = ContactTransitionInput {
            transition_time_lower_s: transition_time_s[0],
            transition_time_upper_s: transition_time_s[1],
            restitution_upper,
            contact_witnesses,
            generalized_acceleration,
            impulse_velocity_response,
        };
        write_contact_transition_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid contact-transition bound: {error:?}"))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_contact_transition_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .expect("validated contact-transition bound");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "contact-transition bound allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Bound a generalized velocity jump with an independent componentwise
    /// continuous-acceleration interval around the contact-impulse tube.
    /// Returned timing is `(elapsed_ns, allocation_calls, allocated_bytes)`.
    #[allow(clippy::too_many_arguments)]
    fn bound_contact_transition_velocity_jump_with_acceleration_interval(
        &self,
        transition_time_s: PyReadonlyArray1<'_, f64>,
        restitution_upper: f64,
        contact_witnesses: PyReadonlyArray2<'_, f64>,
        generalized_acceleration_lower: PyReadonlyArray1<'_, f64>,
        generalized_acceleration_upper: PyReadonlyArray1<'_, f64>,
        impulse_velocity_response: PyReadonlyArray3<'_, f64>,
        mut impulse_upper_out: PyReadwriteArray2<'_, f64>,
        mut delta_velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut delta_velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let witness_shape = contact_witnesses.as_array().dim();
        let response_shape = impulse_velocity_response.as_array().dim();
        let impulse_shape = impulse_upper_out.as_array().dim();
        let transition_time_s = transition_time_s.as_slice()?;
        let contact_witnesses = contact_witnesses.as_slice()?;
        let generalized_acceleration_lower = generalized_acceleration_lower.as_slice()?;
        let generalized_acceleration_upper = generalized_acceleration_upper.as_slice()?;
        let impulse_velocity_response = impulse_velocity_response.as_slice()?;
        let impulse_upper_out = impulse_upper_out.as_slice_mut()?;
        let delta_velocity_lower_out = delta_velocity_lower_out.as_slice_mut()?;
        let delta_velocity_upper_out = delta_velocity_upper_out.as_slice_mut()?;
        let dof = generalized_acceleration_lower.len();
        let contacts = witness_shape.0;
        if transition_time_s.len() != 2
            || generalized_acceleration_upper.len() != dof
            || witness_shape.1 != CONTACT_TRANSITION_WITNESS_WIDTH
            || response_shape != (dof, contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || impulse_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || delta_velocity_lower_out.len() != dof
            || delta_velocity_upper_out.len() != dof
        {
            return Err(PyValueError::new_err(
                "contact transition acceleration interval expects time[2], witnesses[C,4], acceleration lower/upper[D], response[D,C,3], impulse_upper[C,3], and lower/upper[D]",
            ));
        }
        let input = ContactTransitionAccelerationIntervalInput {
            transition_time_lower_s: transition_time_s[0],
            transition_time_upper_s: transition_time_s[1],
            restitution_upper,
            contact_witnesses,
            generalized_acceleration_lower,
            generalized_acceleration_upper,
            impulse_velocity_response,
        };
        write_contact_transition_acceleration_interval_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid contact-transition acceleration interval: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_contact_transition_acceleration_interval_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .expect("validated contact-transition acceleration interval");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "contact-transition acceleration interval allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Bound a contact transition with directional slip, effective-mass and
    /// sustained-force witnesses. Tangential impulse is limited by both
    /// Coulomb capacity and the passive slip-arrest impulse.
    #[allow(clippy::too_many_arguments)]
    fn bound_directional_contact_transition_velocity_jump(
        &self,
        transition_time_s: PyReadonlyArray1<'_, f64>,
        restitution_upper: f64,
        contact_witnesses: PyReadonlyArray2<'_, f64>,
        generalized_acceleration_lower: PyReadonlyArray1<'_, f64>,
        generalized_acceleration_upper: PyReadonlyArray1<'_, f64>,
        impulse_velocity_response: PyReadonlyArray3<'_, f64>,
        mut impulse_upper_out: PyReadwriteArray2<'_, f64>,
        mut delta_velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut delta_velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let witness_shape = contact_witnesses.as_array().dim();
        let response_shape = impulse_velocity_response.as_array().dim();
        let impulse_shape = impulse_upper_out.as_array().dim();
        let transition_time_s = transition_time_s.as_slice()?;
        let contact_witnesses = contact_witnesses.as_slice()?;
        let acceleration_lower = generalized_acceleration_lower.as_slice()?;
        let acceleration_upper = generalized_acceleration_upper.as_slice()?;
        let response = impulse_velocity_response.as_slice()?;
        let impulse_upper_out = impulse_upper_out.as_slice_mut()?;
        let delta_velocity_lower_out = delta_velocity_lower_out.as_slice_mut()?;
        let delta_velocity_upper_out = delta_velocity_upper_out.as_slice_mut()?;
        let dof = acceleration_lower.len();
        let contacts = witness_shape.0;
        if transition_time_s.len() != 2
            || acceleration_upper.len() != dof
            || witness_shape.1 != DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH
            || response_shape != (dof, contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || impulse_shape != (contacts, CONTACT_TRANSITION_IMPULSE_WIDTH)
            || delta_velocity_lower_out.len() != dof
            || delta_velocity_upper_out.len() != dof
        {
            return Err(PyValueError::new_err(
                "directional contact transition expects time[2], witnesses[C,10], acceleration lower/upper[D], response[D,C,3], impulse_upper[C,3], and lower/upper[D]",
            ));
        }
        let input = DirectionalContactTransitionInput {
            transition_time_lower_s: transition_time_s[0],
            transition_time_upper_s: transition_time_s[1],
            restitution_upper,
            contact_witnesses,
            generalized_acceleration_lower: acceleration_lower,
            generalized_acceleration_upper: acceleration_upper,
            impulse_velocity_response: response,
        };
        write_directional_contact_transition_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!(
                "invalid directional contact-transition bound: {error:?}"
            ))
        })?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_directional_contact_transition_bounds(
            input,
            impulse_upper_out,
            delta_velocity_lower_out,
            delta_velocity_upper_out,
        )
        .expect("validated directional contact-transition bound");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "directional contact-transition bound allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Solve one fixed-work two-contact impulse through the full Delassus
    /// response. This returns a model prediction, not an outer bound.
    #[allow(clippy::too_many_arguments)]
    fn solve_coupled_contact_impulse(
        &self,
        contact_velocity: PyReadonlyArray2<'_, f64>,
        delassus: PyReadonlyArray2<'_, f64>,
        impulse_upper: PyReadonlyArray2<'_, f64>,
        friction: PyReadonlyArray1<'_, f64>,
        restitution: f64,
        diagonal_regularization_ratio: f64,
        sweeps: usize,
        mut impulse_out: PyReadwriteArray2<'_, f64>,
        mut contact_velocity_after_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let velocity_shape = contact_velocity.as_array().dim();
        let delassus_shape = delassus.as_array().dim();
        let upper_shape = impulse_upper.as_array().dim();
        let impulse_shape = impulse_out.as_array().dim();
        let after_shape = contact_velocity_after_out.as_array().dim();
        let contact_velocity = contact_velocity.as_slice()?;
        let delassus = delassus.as_slice()?;
        let impulse_upper = impulse_upper.as_slice()?;
        let friction = friction.as_slice()?;
        let impulse_out = impulse_out.as_slice_mut()?;
        let contact_velocity_after_out = contact_velocity_after_out.as_slice_mut()?;
        if velocity_shape != (2, 3)
            || delassus_shape != (6, 6)
            || upper_shape != (2, 3)
            || friction.len() != 2
            || impulse_shape != (2, 3)
            || after_shape != (2, 3)
        {
            return Err(PyValueError::new_err(
                "coupled impulse expects velocity[2,3], delassus[6,6], upper[2,3], friction[2], impulse[2,3], and after[2,3]",
            ));
        }
        let input = CoupledContactImpulseInput {
            contact_velocity,
            delassus,
            impulse_upper,
            friction,
            restitution,
            diagonal_regularization_ratio,
            sweeps,
        };
        solve_coupled_contact_impulse(input, impulse_out, contact_velocity_after_out).map_err(
            |error| PyValueError::new_err(format!("invalid coupled contact impulse: {error:?}")),
        )?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        solve_coupled_contact_impulse(input, impulse_out, contact_velocity_after_out)
            .expect("validated coupled contact impulse");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "coupled contact impulse allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    #[getter]
    fn terminal_impact_diagnostic_names(&self) -> [&'static str; 17] {
        [
            "available",
            "time_to_impact_s",
            "vertical_impact_velocity_m_s",
            "vertical_specific_impact_energy_j_kg",
            "terminal_tilt_rad",
            "terminal_angular_rate_rad_s",
            "minimum_terminal_joint_headroom_fraction",
            "maximum_terminal_joint_velocity_utilization",
            "impact_speed_pressure",
            "tilt_pressure",
            "angular_rate_pressure",
            "joint_position_pressure",
            "joint_velocity_pressure",
            "actuator_effort_pressure",
            "admission_pressure",
            "maximum_terminal_harm_pressure",
            "aggregate_score",
        ]
    }

    #[getter]
    fn terminal_impact_selection_diagnostic_names(&self) -> [&'static str; 6] {
        [
            "selected_index",
            "baseline_index",
            "selected_score",
            "baseline_score",
            "maximum_component_regression",
            "maximum_component_improvement",
        ]
    }

    /// Score exactly three caller-owned support-free candidates at their
    /// ballistic impact time and apply the conservative componentwise chooser.
    /// State is `[clearance, vertical_velocity, roll, pitch, roll_rate,
    /// pitch_rate]`; candidate root acceleration is `[roll, pitch]`.
    #[allow(clippy::too_many_arguments)]
    fn score_terminal_impact_candidates(
        &self,
        state: PyReadonlyArray1<'_, f64>,
        joint_position: PyReadonlyArray1<'_, f64>,
        joint_velocity: PyReadonlyArray1<'_, f64>,
        joint_position_lower: PyReadonlyArray1<'_, f64>,
        joint_position_upper: PyReadonlyArray1<'_, f64>,
        joint_velocity_limit: PyReadonlyArray1<'_, f64>,
        candidate_available: PyReadonlyArray1<'_, u8>,
        candidate_root_angular_acceleration: PyReadonlyArray2<'_, f64>,
        candidate_joint_acceleration: PyReadonlyArray2<'_, f64>,
        candidate_maximum_actuator_effort_utilization: PyReadonlyArray1<'_, f64>,
        baseline_index: usize,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut diagnostics_out: PyReadwriteArray2<'_, f64>,
        mut selection_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        let root_shape = candidate_root_angular_acceleration.as_array().dim();
        let joint_shape = candidate_joint_acceleration.as_array().dim();
        let diagnostic_shape = diagnostics_out.as_array().dim();
        let state = state.as_slice()?;
        let joint_position = joint_position.as_slice()?;
        let joint_velocity = joint_velocity.as_slice()?;
        let joint_position_lower = joint_position_lower.as_slice()?;
        let joint_position_upper = joint_position_upper.as_slice()?;
        let joint_velocity_limit = joint_velocity_limit.as_slice()?;
        let available = candidate_available.as_slice()?;
        let root_acceleration = candidate_root_angular_acceleration.as_slice()?;
        let joint_acceleration = candidate_joint_acceleration.as_slice()?;
        let effort = candidate_maximum_actuator_effort_utilization.as_slice()?;
        let diagnostics = diagnostics_out.as_slice_mut()?;
        let selection_out = selection_out.as_slice_mut()?;
        let joints = joint_position.len();
        if state.len() != 6
            || joint_velocity.len() != joints
            || joint_position_lower.len() != joints
            || joint_position_upper.len() != joints
            || joint_velocity_limit.len() != joints
            || available.len() != CANDIDATES
            || available.iter().any(|value| *value > 1)
            || effort.len() != CANDIDATES
            || root_shape != (CANDIDATES, 2)
            || joint_shape != (CANDIDATES, joints)
            || diagnostic_shape != (CANDIDATES, 17)
            || selection_out.len() != 6
        {
            return Err(PyValueError::new_err(
                "terminal impact expects state[6], joint vectors[J], availability/effort[3], root acceleration[3,2], joint acceleration[3,J], diagnostics[3,17], and selection[6]",
            ));
        }
        let impact_state = TerminalImpactState {
            root_clearance_m: state[0],
            root_vertical_velocity_m_s: state[1],
            root_tilt_rad: [state[2], state[3]],
            root_angular_rate_rad_s: [state[4], state[5]],
            joint_position_rad: joint_position,
            joint_velocity_rad_s: joint_velocity,
            joint_position_lower_rad: joint_position_lower,
            joint_position_upper_rad: joint_position_upper,
            joint_velocity_limit_rad_s: joint_velocity_limit,
        };
        let candidate_at = |index: usize| TerminalImpactCandidate {
            available: available[index] != 0,
            root_angular_acceleration_rad_s2: [
                root_acceleration[2 * index],
                root_acceleration[2 * index + 1],
            ],
            joint_acceleration_rad_s2: &joint_acceleration[index * joints..(index + 1) * joints],
            maximum_actuator_effort_utilization: effort[index],
        };
        let config = TerminalImpactConfig::default();
        let mut scores = [TerminalImpactScore::default(); CANDIDATES];
        for candidate in 0..CANDIDATES {
            scores[candidate] =
                score_terminal_impact(impact_state, candidate_at(candidate), config).map_err(
                    |error| {
                        PyValueError::new_err(format!(
                            "invalid terminal-impact candidate {candidate}: {error:?}"
                        ))
                    },
                )?;
        }
        select_conservative_terminal_impact_candidate(
            &scores,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid terminal-impact selection: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for candidate in 0..CANDIDATES {
            scores[candidate] =
                score_terminal_impact(impact_state, candidate_at(candidate), config)
                    .expect("validated terminal-impact candidate");
        }
        let selection = select_conservative_terminal_impact_candidate(
            &scores,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .expect("validated terminal-impact selection");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "terminal-impact score/selection allocated inside the Rust hot path",
            ));
        }
        for (candidate, score) in scores.iter().enumerate() {
            diagnostics[candidate * 17..(candidate + 1) * 17].copy_from_slice(&[
                f64::from(score.available),
                score.time_to_impact_s,
                score.vertical_impact_velocity_m_s,
                score.vertical_specific_impact_energy_j_kg,
                score.terminal_tilt_rad,
                score.terminal_angular_rate_rad_s,
                score.minimum_terminal_joint_headroom_fraction,
                score.maximum_terminal_joint_velocity_utilization,
                score.impact_speed_pressure,
                score.tilt_pressure,
                score.angular_rate_pressure,
                score.joint_position_pressure,
                score.joint_velocity_pressure,
                score.actuator_effort_pressure,
                score.admission_pressure,
                score.maximum_terminal_harm_pressure,
                score.aggregate_score,
            ]);
        }
        selection_out.copy_from_slice(&[
            selection.selected_index as f64,
            selection.baseline_index as f64,
            selection.selected_score,
            selection.baseline_score,
            selection.maximum_component_regression,
            selection.maximum_component_improvement,
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Aggregate exactly four support hypotheses for each of three typed
    /// terminal candidates, then apply the same conservative chooser. The
    /// hypothesis table is candidate-major `[3, 4, 17]` and contains outputs
    /// from `score_terminal_impact_candidates`.
    #[allow(clippy::too_many_arguments)]
    fn select_terminal_impact_hypothesis_envelopes(
        &self,
        hypothesis_diagnostics: PyReadonlyArray3<'_, f64>,
        baseline_index: usize,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut envelope_diagnostics_out: PyReadwriteArray2<'_, f64>,
        mut selection_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        const HYPOTHESES: usize = 4;
        const DIAGNOSTICS: usize = 17;
        let hypothesis_shape = hypothesis_diagnostics.as_array().dim();
        let envelope_shape = envelope_diagnostics_out.as_array().dim();
        let hypothesis_diagnostics = hypothesis_diagnostics.as_slice()?;
        let envelope_diagnostics = envelope_diagnostics_out.as_slice_mut()?;
        let selection_out = selection_out.as_slice_mut()?;
        if hypothesis_shape != (CANDIDATES, HYPOTHESES, DIAGNOSTICS)
            || envelope_shape != (CANDIDATES, DIAGNOSTICS)
            || selection_out.len() != 6
        {
            return Err(PyValueError::new_err(
                "terminal hypothesis aggregation expects hypotheses[3,4,17], envelopes[3,17], and selection[6]",
            ));
        }
        let mut scores = [TerminalImpactScore::default(); CANDIDATES * HYPOTHESES];
        for (index, values) in hypothesis_diagnostics.chunks_exact(DIAGNOSTICS).enumerate() {
            scores[index] = terminal_impact_score_from_diagnostics(values).ok_or_else(|| {
                PyValueError::new_err(format!(
                    "invalid terminal hypothesis diagnostics at flat index {index}"
                ))
            })?;
        }
        let mut envelopes = [TerminalImpactScore::default(); CANDIDATES];
        write_terminal_impact_hypothesis_envelopes(&scores, HYPOTHESES, &mut envelopes).map_err(
            |error| {
                PyValueError::new_err(format!("invalid terminal hypothesis envelope: {error:?}"))
            },
        )?;
        select_conservative_terminal_impact_candidate(
            &envelopes,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid terminal envelope selection: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        for (index, values) in hypothesis_diagnostics.chunks_exact(DIAGNOSTICS).enumerate() {
            scores[index] = terminal_impact_score_from_diagnostics(values)
                .expect("validated terminal hypothesis diagnostics");
        }
        write_terminal_impact_hypothesis_envelopes(&scores, HYPOTHESES, &mut envelopes)
            .expect("validated terminal hypothesis envelope");
        let selection = select_conservative_terminal_impact_candidate(
            &envelopes,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .expect("validated terminal envelope selection");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "terminal hypothesis aggregation allocated inside the Rust hot path",
            ));
        }
        for (candidate, score) in envelopes.iter().copied().enumerate() {
            write_terminal_impact_score_diagnostics(
                score,
                &mut envelope_diagnostics[candidate * DIAGNOSTICS..(candidate + 1) * DIAGNOSTICS],
            );
        }
        selection_out.copy_from_slice(&[
            selection.selected_index as f64,
            selection.baseline_index as f64,
            selection.selected_score,
            selection.baseline_score,
            selection.maximum_component_regression,
            selection.maximum_component_improvement,
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Aggregate exactly four *paired* terminal hypotheses for each of three
    /// candidates and select from the candidate-minus-baseline consequence
    /// delta.  The hypothesis index is shared across candidates: candidate
    /// `c,h` is compared with baseline `baseline_index,h`, so baseline and
    /// candidate uncertainty is never widened into independent boxes.
    ///
    /// The input table is candidate-major `[3, 4, 17]` and contains the
    /// caller-owned terminal diagnostics for each paired hypothesis.  The
    /// six output delta components are tilt, angular rate, joint position,
    /// joint velocity, actuator effort pressure, and raw joint-headroom loss.
    /// Impact speed is candidate-invariant and availability is the admission
    /// gate. Negative values are improvements. This method is a bounded
    /// consequence selector only; it does not apply a command or admit
    /// authority.
    #[allow(clippy::too_many_arguments)]
    fn select_terminal_impact_delta_hypothesis_envelopes(
        &self,
        hypothesis_diagnostics: PyReadonlyArray3<'_, f64>,
        baseline_index: usize,
        maximum_component_regression: f64,
        minimum_component_improvement: f64,
        mut delta_lower_out: PyReadwriteArray2<'_, f64>,
        mut delta_upper_out: PyReadwriteArray2<'_, f64>,
        mut aggregate_lower_out: PyReadwriteArray1<'_, f64>,
        mut aggregate_upper_out: PyReadwriteArray1<'_, f64>,
        mut selection_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        const CANDIDATES: usize = 3;
        const HYPOTHESES: usize = 4;
        const DIAGNOSTICS: usize = 17;
        const COMPONENTS: usize = TERMINAL_IMPACT_PAIRED_COMPONENTS;
        let hypothesis_shape = hypothesis_diagnostics.as_array().dim();
        let delta_lower_shape = delta_lower_out.as_array().dim();
        let delta_upper_shape = delta_upper_out.as_array().dim();
        let hypothesis_diagnostics = hypothesis_diagnostics.as_slice()?;
        let delta_lower_out = delta_lower_out.as_slice_mut()?;
        let delta_upper_out = delta_upper_out.as_slice_mut()?;
        let aggregate_lower_out = aggregate_lower_out.as_slice_mut()?;
        let aggregate_upper_out = aggregate_upper_out.as_slice_mut()?;
        let selection_out = selection_out.as_slice_mut()?;
        if hypothesis_shape != (CANDIDATES, HYPOTHESES, DIAGNOSTICS)
            || delta_lower_shape != (CANDIDATES, COMPONENTS)
            || delta_upper_shape != (CANDIDATES, COMPONENTS)
            || aggregate_lower_out.len() != CANDIDATES
            || aggregate_upper_out.len() != CANDIDATES
            || selection_out.len() != 6
            || baseline_index >= CANDIDATES
        {
            return Err(PyValueError::new_err(
                "paired terminal delta expects hypotheses[3,4,17], delta lower/upper[3,6], aggregate lower/upper[3], and selection[6]",
            ));
        }

        // Build and validate all stack-owned envelopes before touching any
        // caller output.  The same pass is repeated inside the timed region
        // after validation, keeping the hot path allocation-free and the ABI
        // atomic on malformed late rows.
        let build_envelopes = || -> PyResult<(
            [TerminalImpactComponentDeltaBox; CANDIDATES],
            [TerminalImpactScore; CANDIDATES * HYPOTHESES],
        )> {
            let mut scores =
                [TerminalImpactScore::default(); CANDIDATES * HYPOTHESES];
            for (index, values) in hypothesis_diagnostics
                .chunks_exact(DIAGNOSTICS)
                .enumerate()
            {
                scores[index] = terminal_impact_score_from_diagnostics(values).ok_or_else(|| {
                    PyValueError::new_err(format!(
                        "invalid paired terminal hypothesis diagnostics at flat index {index}"
                    ))
                })?;
            }
            let mut envelopes = [TerminalImpactComponentDeltaBox {
                available: false,
                component_lower: [0.0; COMPONENTS],
                component_upper: [0.0; COMPONENTS],
                aggregate_lower: 0.0,
                aggregate_upper: 0.0,
            }; CANDIDATES];
            for candidate in 0..CANDIDATES {
                if candidate == baseline_index {
                    // The baseline's paired delta is exactly zero by
                    // definition, regardless of any repeated floating bits.
                    envelopes[candidate].available = (0..HYPOTHESES)
                        .all(|hypothesis| {
                            scores[candidate * HYPOTHESES + hypothesis].available
                        });
                    continue;
                }
                let mut lower = [f64::INFINITY; COMPONENTS];
                let mut upper = [f64::NEG_INFINITY; COMPONENTS];
                let mut aggregate_lower = f64::INFINITY;
                let mut aggregate_upper = f64::NEG_INFINITY;
                let mut available = true;
                for hypothesis in 0..HYPOTHESES {
                    let baseline = scores[baseline_index * HYPOTHESES + hypothesis];
                    let score = scores[candidate * HYPOTHESES + hypothesis];
                    available &= baseline.available && score.available;
                    let deltas = [
                        score.tilt_pressure - baseline.tilt_pressure,
                        score.angular_rate_pressure - baseline.angular_rate_pressure,
                        score.joint_position_pressure - baseline.joint_position_pressure,
                        score.joint_velocity_pressure - baseline.joint_velocity_pressure,
                        score.actuator_effort_pressure - baseline.actuator_effort_pressure,
                        baseline.minimum_terminal_joint_headroom_fraction
                            - score.minimum_terminal_joint_headroom_fraction,
                    ];
                    if deltas.iter().any(|value| !value.is_finite()) {
                        return Err(PyValueError::new_err(
                            "paired terminal delta overflowed a finite component",
                        ));
                    }
                    for component in 0..COMPONENTS {
                        lower[component] = lower[component].min(deltas[component]);
                        upper[component] = upper[component].max(deltas[component]);
                    }
                    let aggregate = score.aggregate_score - baseline.aggregate_score;
                    if !aggregate.is_finite() {
                        return Err(PyValueError::new_err(
                            "paired terminal delta overflowed a finite aggregate",
                        ));
                    }
                    aggregate_lower = aggregate_lower.min(aggregate);
                    aggregate_upper = aggregate_upper.max(aggregate);
                }
                envelopes[candidate] = TerminalImpactComponentDeltaBox {
                    available,
                    component_lower: lower,
                    component_upper: upper,
                    aggregate_lower,
                    aggregate_upper,
                };
            }
            envelopes[baseline_index] = TerminalImpactComponentDeltaBox {
                available: envelopes[baseline_index].available,
                component_lower: [0.0; COMPONENTS],
                component_upper: [0.0; COMPONENTS],
                aggregate_lower: 0.0,
                aggregate_upper: 0.0,
            };
            select_conservative_terminal_impact_delta_candidate(
                &envelopes,
                baseline_index,
                maximum_component_regression,
                minimum_component_improvement,
            )
            .map_err(|error| {
                PyValueError::new_err(format!(
                    "invalid paired terminal delta selection: {error:?}"
                ))
            })?;
            Ok((envelopes, scores))
        };

        let _ = build_envelopes()?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        let (envelopes, _scores) = build_envelopes()?;
        let selection = select_conservative_terminal_impact_delta_candidate(
            &envelopes,
            baseline_index,
            maximum_component_regression,
            minimum_component_improvement,
        )
        .expect("validated paired terminal delta selection");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "paired terminal delta selection allocated inside the Rust hot path",
            ));
        }
        for candidate in 0..CANDIDATES {
            for component in 0..COMPONENTS {
                delta_lower_out[candidate * COMPONENTS + component] =
                    envelopes[candidate].component_lower[component];
                delta_upper_out[candidate * COMPONENTS + component] =
                    envelopes[candidate].component_upper[component];
            }
            aggregate_lower_out[candidate] = envelopes[candidate].aggregate_lower;
            aggregate_upper_out[candidate] = envelopes[candidate].aggregate_upper;
        }
        selection_out.copy_from_slice(&[
            selection.selected_index as f64,
            selection.baseline_index as f64,
            selection.maximum_component_delta_upper,
            selection.maximum_guaranteed_component_improvement,
            selection.aggregate_delta_lower,
            selection.aggregate_delta_upper,
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Install the terminal chooser's retained-effort decision without
    /// resetting contact-program state or extending its independent age
    /// budget. Candidate zero is withhold, one is retained effort, and two is
    /// the separately admitted support-free command.
    fn install_contact_program_terminal_impact_selection(
        &mut self,
        selected_index: usize,
    ) -> PyResult<()> {
        if selected_index > 2 {
            return Err(PyValueError::new_err(
                "terminal-impact selection index must be in 0..=2",
            ));
        }
        self.contact_program_authority_config
            .inexact_observation_hold_authority_q15 = if selected_index == 1 { 32_768 } else { 0 };
        Ok(())
    }

    /// Admit one caller-timestamped two-wheel contact observation. The Rust
    /// filter owns source/sequence/time validation and debounce state. Held,
    /// missing, or rejected evidence never emits a hard contact mask.
    #[allow(clippy::too_many_arguments)]
    fn step_contact_observation_from_mask(
        &mut self,
        tick_time_ns: u64,
        observation_available: bool,
        mapped_time_ns: u64,
        source_sequence: u64,
        source_identity: u32,
        synchronization_uncertainty_ns: u64,
        raw_contact: PyReadonlyArray1<'_, u8>,
        mut debounced_contact_out: PyReadwriteArray1<'_, u8>,
        mut hard_contact_out: PyReadwriteArray1<'_, u8>,
        mut diagnostics_out: PyReadwriteArray1<'_, i64>,
    ) -> PyResult<()> {
        let raw_contact = raw_contact.as_slice()?;
        let debounced_contact_out = debounced_contact_out.as_slice_mut()?;
        let hard_contact_out = hard_contact_out.as_slice_mut()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        if raw_contact.len() != 2
            || debounced_contact_out.len() != 2
            || hard_contact_out.len() != 2
            || diagnostics_out.len() != 16
            || raw_contact.iter().any(|value| *value > 1)
        {
            return Err(PyValueError::new_err(
                "contact observation expects raw/debounced/hard masks of length two with binary values and diagnostics[16]",
            ));
        }
        let observation = observation_available.then_some(ContactObservation {
            mapped_time_ns,
            source_sequence,
            source_identity,
            synchronization_uncertainty_ns,
            active: [raw_contact[0] != 0, raw_contact[1] != 0],
        });
        let allocation_before = allocation_snapshot();
        let output = step_contact_observation(
            tick_time_ns,
            observation,
            self.contact_observation_config,
            &mut self.contact_observation_state,
        );
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "contact observation step allocated inside the Rust hot path",
            ));
        }
        for contact in 0..2 {
            debounced_contact_out[contact] = u8::from(output.debounced_contact[contact]);
            hard_contact_out[contact] = u8::from(output.hard_contact[contact]);
        }
        let saturating_i64 = |value: u64| i64::try_from(value).unwrap_or(i64::MAX);
        let mask = |values: [bool; 2]| i64::from(values[0]) | (i64::from(values[1]) << 1);
        diagnostics_out.copy_from_slice(&[
            output.status as u8 as i64,
            output.provenance as u8 as i64,
            i64::from(output.accepted),
            i64::from(output.hard_constraint_eligible),
            saturating_i64(output.age_ns),
            mask(output.raw_contact),
            mask(output.debounced_contact),
            mask(output.hard_contact),
            i64::from(output.pending_samples[0]),
            i64::from(output.pending_samples[1]),
            i64::from(output.transition_count),
            i64::from(output.flags),
            saturating_i64(mapped_time_ns),
            saturating_i64(source_sequence),
            i64::from(source_identity),
            saturating_i64(synchronization_uncertainty_ns),
        ]);
        Ok(())
    }

    /// Retain one admitted six-actuator command across a bounded exact contact
    /// transition. Rust owns the command bytes, authoring support mask, tick
    /// age, and revocation state; Python owns only experiment sequencing.
    #[allow(clippy::too_many_arguments)]
    fn step_contact_command_lease_from_masks(
        &mut self,
        tick_sequence: u64,
        observation_exact: bool,
        stable_contact: PyReadonlyArray1<'_, u8>,
        hard_contact: PyReadonlyArray1<'_, u8>,
        fresh_command_available: bool,
        fresh_command: PyReadonlyArray1<'_, f64>,
        mut command_out: PyReadwriteArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, i64>,
    ) -> PyResult<()> {
        let stable_contact = stable_contact.as_slice()?;
        let hard_contact = hard_contact.as_slice()?;
        let fresh_command = fresh_command.as_slice()?;
        let command_out = command_out.as_slice_mut()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        if stable_contact.len() != 2
            || hard_contact.len() != 2
            || fresh_command.len() != 6
            || command_out.len() != 6
            || diagnostics_out.len() != 14
            || stable_contact.iter().any(|value| *value > 1)
            || hard_contact.iter().any(|value| *value > 1)
        {
            return Err(PyValueError::new_err(
                "contact command lease expects binary stable/hard masks[2], command input/output[6], and diagnostics[14]",
            ));
        }
        let candidate = fresh_command_available.then_some([
            fresh_command[0],
            fresh_command[1],
            fresh_command[2],
            fresh_command[3],
            fresh_command[4],
            fresh_command[5],
        ]);
        let stable = [stable_contact[0] != 0, stable_contact[1] != 0];
        let hard = [hard_contact[0] != 0, hard_contact[1] != 0];
        let allocation_before = allocation_snapshot();
        let output = step_contact_command_lease(
            tick_sequence,
            observation_exact,
            stable,
            hard,
            candidate,
            self.contact_command_lease_config,
            &mut self.contact_command_lease_state,
        );
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "contact command lease allocated inside the Rust hot path",
            ));
        }
        command_out.copy_from_slice(&output.command);
        let saturating_i64 = |value: u64| i64::try_from(value).unwrap_or(i64::MAX);
        let mask = |values: [bool; 2]| i64::from(values[0]) | (i64::from(values[1]) << 1);
        diagnostics_out.copy_from_slice(&[
            output.status as u8 as i64,
            output.provenance as u8 as i64,
            i64::from(output.executable),
            saturating_i64(output.command_age_ticks),
            i64::from(output.remaining_hold_ticks),
            mask(output.authoring_contact),
            mask(output.stable_contact),
            mask(output.hard_contact),
            i64::from(output.transition_count),
            i64::from(output.flags),
            saturating_i64(tick_sequence),
            i64::from(self.contact_command_lease_config.maximum_hold_ticks),
            i64::from(observation_exact),
            i64::from(fresh_command_available),
        ]);
        Ok(())
    }

    /// Configure the generic primary/current-support/lease authority combiner.
    #[pyo3(signature = (
        maximum_hold_ticks,
        allow_transition_current_support=false,
        maximum_inexact_observation_hold_ticks=0,
        inexact_observation_hold_authority=1.0
    ))]
    fn configure_contact_program_authority(
        &mut self,
        maximum_hold_ticks: u32,
        allow_transition_current_support: bool,
        maximum_inexact_observation_hold_ticks: u32,
        inexact_observation_hold_authority: f64,
    ) -> PyResult<()> {
        if !inexact_observation_hold_authority.is_finite()
            || !(0.0..=1.0).contains(&inexact_observation_hold_authority)
        {
            return Err(PyValueError::new_err(
                "inexact observation hold authority must be finite in [0, 1]",
            ));
        }
        self.contact_program_authority_config.maximum_hold_ticks = maximum_hold_ticks;
        self.contact_program_authority_config
            .allow_transition_current_support = allow_transition_current_support;
        self.contact_program_authority_config
            .maximum_inexact_observation_hold_ticks = maximum_inexact_observation_hold_ticks;
        self.contact_program_authority_config
            .inexact_observation_hold_authority_q15 = (inexact_observation_hold_authority
            * f64::from(
                bonesaw_core::contact_program_authority::INEXACT_OBSERVATION_HOLD_AUTHORITY_ONE_Q15,
            ))
        .round() as u16;
        self.contact_program_authority_state = ContactProgramAuthorityState::default();
        Ok(())
    }

    #[getter]
    fn contact_program_authority_diagnostic_names(&self) -> [&'static str; 23] {
        [
            "selection",
            "executable",
            "transition_pending",
            "activation_pending",
            "deactivation_pending",
            "masks_consistent",
            "lease_status",
            "lease_provenance",
            "command_age_ticks",
            "remaining_hold_ticks",
            "authoring_mask",
            "stable_mask",
            "hard_mask",
            "transition_count",
            "flags",
            "tick_sequence",
            "maximum_hold_ticks",
            "observation_exact",
            "primary_command_available",
            "current_support_command_available",
            "allow_transition_current_support",
            "maximum_inexact_observation_hold_ticks",
            "inexact_observation_hold_authority_q15",
        ]
    }

    /// Combine already-admitted primary and current-support commands. Fresh
    /// primary authority requires raw/stable/hard masks to agree. With the
    /// explicit opt-in, a separately admitted current-hard-support command may
    /// become fresh during transition after prior authority has existed.
    #[allow(clippy::too_many_arguments)]
    fn step_contact_program_authority_from_masks(
        &mut self,
        tick_sequence: u64,
        observation_exact: bool,
        raw_contact: PyReadonlyArray1<'_, u8>,
        stable_contact: PyReadonlyArray1<'_, u8>,
        hard_contact: PyReadonlyArray1<'_, u8>,
        primary_command_available: bool,
        primary_command: PyReadonlyArray1<'_, f64>,
        current_support_command_available: bool,
        current_support_command: PyReadonlyArray1<'_, f64>,
        support_free_inexact_observation_command_available: bool,
        support_free_inexact_observation_command: PyReadonlyArray1<'_, f64>,
        mut command_out: PyReadwriteArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, i64>,
    ) -> PyResult<()> {
        let raw_contact = raw_contact.as_slice()?;
        let stable_contact = stable_contact.as_slice()?;
        let hard_contact = hard_contact.as_slice()?;
        let primary_command = primary_command.as_slice()?;
        let current_support_command = current_support_command.as_slice()?;
        let support_free_inexact_observation_command =
            support_free_inexact_observation_command.as_slice()?;
        let command_out = command_out.as_slice_mut()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        if raw_contact.len() != 2
            || stable_contact.len() != 2
            || hard_contact.len() != 2
            || primary_command.len() != 6
            || current_support_command.len() != 6
            || support_free_inexact_observation_command.len() != 6
            || command_out.len() != 6
            || diagnostics_out.len() != 23
            || raw_contact.iter().any(|value| *value > 1)
            || stable_contact.iter().any(|value| *value > 1)
            || hard_contact.iter().any(|value| *value > 1)
        {
            return Err(PyValueError::new_err(
                "contact program authority expects binary raw/stable/hard masks[2], primary/current/support-free-inexact/output commands[6], and diagnostics[23]",
            ));
        }
        let command = |values: &[f64]| {
            [
                values[0], values[1], values[2], values[3], values[4], values[5],
            ]
        };
        let raw = [raw_contact[0] != 0, raw_contact[1] != 0];
        let stable = [stable_contact[0] != 0, stable_contact[1] != 0];
        let hard = [hard_contact[0] != 0, hard_contact[1] != 0];
        let allocation_before = allocation_snapshot();
        let output = step_contact_program_authority_with_inexact_command(
            tick_sequence,
            observation_exact,
            raw,
            stable,
            hard,
            primary_command_available.then(|| command(primary_command)),
            current_support_command_available.then(|| command(current_support_command)),
            support_free_inexact_observation_command_available
                .then(|| command(support_free_inexact_observation_command)),
            self.contact_program_authority_config,
            &mut self.contact_program_authority_state,
        );
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "contact program authority allocated inside the Rust hot path",
            ));
        }
        command_out.copy_from_slice(&output.lease.command);
        let saturating_i64 = |value: u64| i64::try_from(value).unwrap_or(i64::MAX);
        let mask = |values: [bool; 2]| i64::from(values[0]) | (i64::from(values[1]) << 1);
        diagnostics_out.copy_from_slice(&[
            output.selection as u8 as i64,
            i64::from(output.executable),
            i64::from(output.transition_pending),
            i64::from(output.activation_pending),
            i64::from(output.deactivation_pending),
            i64::from(output.masks_consistent),
            output.lease.status as u8 as i64,
            output.lease.provenance as u8 as i64,
            saturating_i64(output.lease.command_age_ticks),
            i64::from(output.lease.remaining_hold_ticks),
            mask(output.lease.authoring_contact),
            mask(output.lease.stable_contact),
            mask(output.lease.hard_contact),
            i64::from(output.lease.transition_count),
            i64::from(output.lease.flags),
            saturating_i64(tick_sequence),
            i64::from(self.contact_program_authority_config.maximum_hold_ticks),
            i64::from(observation_exact),
            i64::from(primary_command_available),
            i64::from(current_support_command_available),
            i64::from(
                self.contact_program_authority_config
                    .allow_transition_current_support,
            ),
            i64::from(
                self.contact_program_authority_config
                    .maximum_inexact_observation_hold_ticks,
            ),
            i64::from(
                self.contact_program_authority_config
                    .inexact_observation_hold_authority_q15,
            ),
        ]);
        Ok(())
    }

    /// Emit a support-conditioned contingency request from the exact current
    /// support mask. This method performs FK and request generation only; its
    /// output must pass through an independent floating-WBC admission before
    /// it is executable.
    #[allow(clippy::too_many_arguments)]
    fn write_support_contingency_from_state(
        &mut self,
        support_mask: u8,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        root_twist_world: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        joint_velocity: PyReadonlyArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
        mut root_angular_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut root_linear_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut joint_acceleration_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let root_twist_world = root_twist_world.as_slice()?;
        let q = q.as_slice()?;
        let joint_velocity = joint_velocity.as_slice()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        let root_angular_acceleration_out = root_angular_acceleration_out.as_slice_mut()?;
        let root_linear_acceleration_out = root_linear_acceleration_out.as_slice_mut()?;
        let joint_acceleration_out = joint_acceleration_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        if support_mask > 3
            || root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || root_twist_world.len() != 6
            || q.len() != dof
            || joint_velocity.len() != dof
            || diagnostics_out.len() != 17
            || root_angular_acceleration_out.len() != 3
            || root_linear_acceleration_out.len() != 3
            || joint_acceleration_out.len() != dof
        {
            return Err(PyValueError::new_err(format!(
                "support contingency expects mask in [0,3], root[3], quaternion[4], twist[6], q/v[{dof}], diagnostics[17], root angular/linear[3], and joint[{dof}] outputs"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(root_twist_world)
            .chain(q)
            .chain(joint_velocity)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "support contingency state must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("support contingency quaternion is degenerate"))?;
        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        self.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.robot.q.as_mut_slice().copy_from_slice(q);
        self.robot.v.as_mut_slice().copy_from_slice(joint_velocity);
        self.program
            .model
            .forward_kinematics(&self.robot, &mut self.cache)
            .map_err(value_error)?;
        let support_positions = self.support_frames.map(|frame| {
            self.cache.world_from_body[frame.0]
                .transform_point(&Point3::origin())
                .coords
        });
        let active_support_centroid = match support_mask {
            1 => support_positions[0],
            2 => support_positions[1],
            3 => 0.5 * (support_positions[0] + support_positions[1]),
            _ => Vec3::zeros(),
        };
        let root_twist_world: &[f64; 6] = root_twist_world
            .try_into()
            .expect("root twist length was validated");
        let root_angular_acceleration_out: &mut [f64; 3] = root_angular_acceleration_out
            .try_into()
            .expect("root angular output length was validated");
        let root_linear_acceleration_out: &mut [f64; 3] = root_linear_acceleration_out
            .try_into()
            .expect("root linear output length was validated");
        let evidence = write_support_contingency_request(
            support_mask,
            self.cache.center_of_mass_world,
            active_support_centroid,
            rotation.scaled_axis(),
            root_twist_world,
            joint_velocity,
            self.support_contingency_config,
            root_angular_acceleration_out,
            root_linear_acceleration_out,
            joint_acceleration_out,
        )
        .ok_or_else(|| PyValueError::new_err("support contingency request is invalid"))?;
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        diagnostics_out.copy_from_slice(&[
            evidence.mode as u8 as f64,
            evidence.support_mask as f64,
            evidence.active_support_count as f64,
            evidence.center_of_mass_error_world_m.x,
            evidence.center_of_mass_error_world_m.y,
            evidence.center_of_mass_error_world_m.z,
            evidence.requested_horizontal_acceleration_world_m_s2.x,
            evidence.requested_horizontal_acceleration_world_m_s2.y,
            evidence.requested_horizontal_acceleration_world_m_s2.z,
            evidence.requested_tilt_rotation_vector_world_rad.x,
            evidence.requested_tilt_rotation_vector_world_rad.y,
            evidence.requested_tilt_rotation_vector_world_rad.z,
            evidence.ballistic_vertical_acceleration as u8 as f64,
            evidence.horizontal_acceleration_was_saturated as u8 as f64,
            evidence.vertical_acceleration_was_saturated as u8 as f64,
            evidence.angular_acceleration_was_saturated as u8 as f64,
            evidence.joint_acceleration_was_saturated as u8 as f64,
        ]);
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Explicit contingency selection for the Upkie example. The caller owns
    /// the observation and supplies the previous WBC admission result. Rust
    /// derives tilt/rate evidence and advances the bounded supervisor state.
    fn step_fall_safe_from_state(
        &mut self,
        timestep_seconds: f64,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        root_twist_world: PyReadonlyArray1<'_, f64>,
        joint_velocity: PyReadonlyArray1<'_, f64>,
        previous_solver_admitted: bool,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
        mut contingency_root_angular_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut contingency_root_linear_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut contingency_joint_acceleration_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<()> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let root_twist_world = root_twist_world.as_slice()?;
        let joint_velocity = joint_velocity.as_slice()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        let contingency_root_angular_acceleration_out =
            contingency_root_angular_acceleration_out.as_slice_mut()?;
        let contingency_root_linear_acceleration_out =
            contingency_root_linear_acceleration_out.as_slice_mut()?;
        let contingency_joint_acceleration_out =
            contingency_joint_acceleration_out.as_slice_mut()?;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || root_twist_world.len() != 6
            || joint_velocity.len() != self.program.model.dof
            || diagnostics_out.len() != 19
            || contingency_root_angular_acceleration_out.len() != 3
            || contingency_root_linear_acceleration_out.len() != 3
            || contingency_joint_acceleration_out.len() != self.program.model.dof
        {
            return Err(PyValueError::new_err(
                "fall-safe step expects root[3], quaternion[4], root twist[6], joint velocity[dof], diagnostics[19], root angular[3], root linear[3], and joint[dof] outputs",
            ));
        }
        if !timestep_seconds.is_finite()
            || timestep_seconds <= 0.0
            || root_position
                .iter()
                .chain(root_quaternion_wxyz)
                .chain(root_twist_world)
                .chain(joint_velocity)
                .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "fall-safe observation must be finite with a positive timestep",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("fall-safe root quaternion is degenerate"))?;
        let rotation_vector = rotation.scaled_axis();
        let tilt_rad = rotation_vector.xy().norm();
        let angular_rate_rad_s = Vec3::from_row_slice(&root_twist_world[..3]).xy().norm();
        let allocation_before = allocation_snapshot();
        let output = step_upkie_fall_safe(
            timestep_seconds,
            tilt_rad,
            angular_rate_rad_s,
            root_position[2],
            previous_solver_admitted,
            self.fall_safe_config,
            &mut self.fall_safe_state,
        )
        .ok_or_else(|| PyValueError::new_err("fall-safe configuration or state is invalid"))?;
        if !write_upkie_fall_safe_contingency(
            root_twist_world,
            joint_velocity,
            self.fall_safe_config,
            contingency_root_angular_acceleration_out,
            contingency_root_linear_acceleration_out,
            contingency_joint_acceleration_out,
        ) {
            return Err(PyValueError::new_err(
                "fall-safe contingency configuration or state is invalid",
            ));
        }
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "fall-safe step allocated inside the Rust hot path",
            ));
        }
        diagnostics_out.copy_from_slice(&[
            output.mode as u8 as f64,
            tilt_rad,
            angular_rate_rad_s,
            root_position[2],
            output.tilt_pressure,
            output.angular_rate_pressure,
            output.height_pressure,
            output.solver_pressure,
            output.raw_risk,
            output.requested_primary_authority,
            output.primary_authority,
            output.contingency_authority,
            output.fresh_command_authority,
            output.hold_remaining_s,
            output.consecutive_nonadmitted_steps as f64,
            output.transition_count as f64,
            output.limiting_reason_flags as f64,
            output.authority_was_slew_limited as u8 as f64,
            output.fallen_latched as u8 as f64,
        ]);
        Ok(())
    }

    fn balanced_standing(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        mut root_position_out: PyReadwriteArray1<'_, f64>,
        mut q_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<f64> {
        let root_position = root_position.as_slice()?;
        let q = q.as_slice()?;
        let root_position_out = root_position_out.as_slice_mut()?;
        let q_out = q_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        if root_position.len() != 3
            || root_position_out.len() != 3
            || q.len() != dof
            || q_out.len() != dof
        {
            return Err(PyValueError::new_err(format!(
                "balanced_standing expects root[3], q[{dof}], root_out[3], and q_out[{dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(q)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "balanced_standing inputs must be finite",
            ));
        }
        self.robot.control_world_from_root = Transform3::identity();
        self.robot.control_world_from_root.translation.vector = Vec3::from_row_slice(root_position);
        self.robot.q.as_mut_slice().copy_from_slice(q);
        self.robot.v.fill(0.0);
        self.program
            .model
            .forward_kinematics(&self.robot, &mut self.cache)
            .map_err(value_error)?;
        let support_targets = self.support_frames.map(|frame| {
            self.cache.world_from_body[frame.0]
                .transform_point(&Point3::origin())
                .coords
        });
        let ik_targets = [
            PlanarPointIkTarget {
                frame: self.support_frames[0],
                point_in_frame: Vec3::zeros(),
                target_world: support_targets[0],
                coordinates: self.leg_coordinates[0],
                axes: [0, 2],
            },
            PlanarPointIkTarget {
                frame: self.support_frames[1],
                point_in_frame: Vec3::zeros(),
                target_world: support_targets[1],
                coordinates: self.leg_coordinates[1],
                axes: [0, 2],
            },
        ];
        let support_center_x = 0.5 * (support_targets[0].x + support_targets[1].x);
        for _ in 0..8 {
            self.program
                .model
                .forward_kinematics(&self.robot, &mut self.cache)
                .map_err(value_error)?;
            let error = self.cache.center_of_mass_world.x - support_center_x;
            if error.abs() <= 1.0e-9 {
                break;
            }
            self.robot.control_world_from_root.translation.vector.x -= error;
            let report = solve_planar_point_ik_into(
                &self.program.model,
                &mut self.robot,
                &ik_targets,
                PlanarIkOptions::default(),
                &mut self.ik_scratch,
            )
            .map_err(value_error)?;
            if !report.converged {
                return Err(PyValueError::new_err(format!(
                    "balanced standing IK did not converge: {:.3} mm",
                    report.maximum_planar_error_m * 1_000.0
                )));
            }
        }
        self.program
            .model
            .forward_kinematics(&self.robot, &mut self.cache)
            .map_err(value_error)?;
        root_position_out.copy_from_slice(
            self.robot
                .control_world_from_root
                .translation
                .vector
                .as_slice(),
        );
        q_out.copy_from_slice(self.robot.q.as_slice());
        Ok(self.cache.center_of_mass_world.x - support_center_x)
    }

    fn step(
        &mut self,
        timestep_seconds: f64,
        target_ground_position: f64,
        ground_position: f64,
        base_pitch: f64,
        joint_velocity: PyReadonlyArray1<'_, f64>,
        mut wheel_acceleration_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<f64> {
        if !timestep_seconds.is_finite() || timestep_seconds <= 0.0 {
            return Err(PyValueError::new_err(
                "timestep_seconds must be finite and positive",
            ));
        }
        if !target_ground_position.is_finite()
            || !ground_position.is_finite()
            || !base_pitch.is_finite()
        {
            return Err(PyValueError::new_err(
                "Upkie balance observations must be finite",
            ));
        }
        let joint_velocity = joint_velocity.as_slice()?;
        let wheel_acceleration_out = wheel_acceleration_out.as_slice_mut()?;
        if joint_velocity.len() != self.joint_velocity.len() || wheel_acceleration_out.len() != 2 {
            return Err(PyValueError::new_err(format!(
                "expected joint_velocity[{}] and wheel_acceleration_out[2]",
                self.joint_velocity.len()
            )));
        }
        self.joint_velocity.copy_from_slice(joint_velocity);
        let mut desired_accelerations = [0.0; 2];
        let allocation_before = allocation_snapshot();
        let ground_velocity = self.balancer.emit_accelerations(
            timestep_seconds,
            target_ground_position,
            ground_position,
            base_pitch,
            &self.joint_velocity,
            &mut self.state,
            &mut desired_accelerations,
        );
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "Upkie balance step allocated inside the Rust hot path",
            ));
        }
        wheel_acceleration_out.copy_from_slice(&desired_accelerations);
        Ok(ground_velocity)
    }

    #[allow(clippy::too_many_arguments)]
    fn step_from_state(
        &mut self,
        timestep_seconds: f64,
        target_ground_position: f64,
        ground_position: f64,
        ground_height: f64,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        joint_velocity: PyReadonlyArray1<'_, f64>,
        mut wheel_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut virtual_pitch_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<()> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let joint_velocity = joint_velocity.as_slice()?;
        let wheel_acceleration_out = wheel_acceleration_out.as_slice_mut()?;
        let virtual_pitch_out = virtual_pitch_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || joint_velocity.len() != dof
            || wheel_acceleration_out.len() != 2
            || virtual_pitch_out.len() != 1
        {
            return Err(PyValueError::new_err(format!(
                "step_from_state expects root[3], quaternion[4], q/v[{dof}], wheel_out[2], and pitch_out[1]"
            )));
        }
        if !timestep_seconds.is_finite()
            || timestep_seconds <= 0.0
            || !target_ground_position.is_finite()
            || !ground_position.is_finite()
            || !ground_height.is_finite()
            || root_position
                .iter()
                .chain(root_quaternion_wxyz)
                .chain(q)
                .chain(joint_velocity)
                .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "Upkie balance state and timestep must be finite, with a positive timestep",
            ));
        }
        let quaternion = nalgebra::Quaternion::new(
            root_quaternion_wxyz[0],
            root_quaternion_wxyz[1],
            root_quaternion_wxyz[2],
            root_quaternion_wxyz[3],
        );
        let rotation = UnitQuaternion::try_new(quaternion, 1.0e-12)
            .ok_or_else(|| PyValueError::new_err("root quaternion is degenerate"))?;
        let allocation_before = allocation_snapshot();
        self.robot.control_world_from_root.rotation = rotation;
        self.robot.control_world_from_root.translation.vector = Vec3::from_row_slice(root_position);
        self.robot.q.as_mut_slice().copy_from_slice(q);
        self.robot.v.as_mut_slice().copy_from_slice(joint_velocity);
        self.joint_velocity.copy_from_slice(joint_velocity);
        self.program
            .model
            .forward_kinematics(&self.robot, &mut self.cache)
            .map_err(value_error)?;
        let virtual_pitch = (self.cache.center_of_mass_world.x - ground_position)
            .atan2((self.cache.center_of_mass_world.z - ground_height).max(0.05));
        let mut desired_accelerations = [0.0; 2];
        self.balancer.emit_accelerations(
            timestep_seconds,
            target_ground_position,
            ground_position,
            virtual_pitch,
            &self.joint_velocity,
            &mut self.state,
            &mut desired_accelerations,
        );
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "Upkie balance state step allocated inside the Rust hot path",
            ));
        }
        wheel_acceleration_out.copy_from_slice(&desired_accelerations);
        virtual_pitch_out[0] = virtual_pitch;
        Ok(())
    }

    /// Rooted sagittal capture reference for the Upkie example. `map_from_odom`
    /// is evaluated for reporting only; only the smooth `control_world <- odom`
    /// transform may affect the lower-priority station target.
    #[allow(clippy::too_many_arguments)]
    fn step_rooted_capture_from_state(
        &mut self,
        timestep_seconds: f64,
        station_ground_position_odom_m: f64,
        ground_height_control_world_m: f64,
        control_world_from_odom_translation: PyReadonlyArray1<'_, f64>,
        control_world_from_odom_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        map_from_odom_translation: PyReadonlyArray1<'_, f64>,
        map_from_odom_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        root_twist_world: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        joint_velocity: PyReadonlyArray1<'_, f64>,
        mut wheel_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<()> {
        let control_world_from_odom_translation = control_world_from_odom_translation.as_slice()?;
        let control_world_from_odom_quaternion_wxyz =
            control_world_from_odom_quaternion_wxyz.as_slice()?;
        let map_from_odom_translation = map_from_odom_translation.as_slice()?;
        let map_from_odom_quaternion_wxyz = map_from_odom_quaternion_wxyz.as_slice()?;
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let root_twist_world = root_twist_world.as_slice()?;
        let q = q.as_slice()?;
        let joint_velocity = joint_velocity.as_slice()?;
        let wheel_acceleration_out = wheel_acceleration_out.as_slice_mut()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let valid_shapes = control_world_from_odom_translation.len() == 3
            && control_world_from_odom_quaternion_wxyz.len() == 4
            && map_from_odom_translation.len() == 3
            && map_from_odom_quaternion_wxyz.len() == 4
            && root_position.len() == 3
            && root_quaternion_wxyz.len() == 4
            && root_twist_world.len() == 6
            && q.len() == dof
            && joint_velocity.len() == dof
            && wheel_acceleration_out.len() == 2
            && diagnostics_out.len() == 16;
        if !valid_shapes {
            return Err(PyValueError::new_err(format!(
                "rooted capture expects translations[3], quaternions[4], root twist[6], q/v[{dof}], wheel_out[2], and diagnostics_out[16]"
            )));
        }
        if !timestep_seconds.is_finite()
            || timestep_seconds <= 0.0
            || !station_ground_position_odom_m.is_finite()
            || !ground_height_control_world_m.is_finite()
            || control_world_from_odom_translation
                .iter()
                .chain(control_world_from_odom_quaternion_wxyz)
                .chain(map_from_odom_translation)
                .chain(map_from_odom_quaternion_wxyz)
                .chain(root_position)
                .chain(root_quaternion_wxyz)
                .chain(root_twist_world)
                .chain(q)
                .chain(joint_velocity)
                .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "rooted capture state must be finite with a positive timestep",
            ));
        }
        let transform = |translation: &[f64], quaternion: &[f64]| -> PyResult<Transform3> {
            let rotation = UnitQuaternion::try_new(
                nalgebra::Quaternion::new(
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                ),
                1.0e-12,
            )
            .ok_or_else(|| PyValueError::new_err("rooted capture quaternion is degenerate"))?;
            Ok(Transform3::from_parts(
                Translation3::new(translation[0], translation[1], translation[2]),
                rotation,
            ))
        };
        let control_world_from_odom = transform(
            control_world_from_odom_translation,
            control_world_from_odom_quaternion_wxyz,
        )?;
        let map_from_odom = transform(map_from_odom_translation, map_from_odom_quaternion_wxyz)?;
        let control_world_from_root = transform(root_position, root_quaternion_wxyz)?;
        let wheel_acceleration: &mut [f64; 2] = wheel_acceleration_out
            .try_into()
            .expect("wheel acceleration length was validated");

        let allocation_before = allocation_snapshot();
        self.robot.control_world_from_root = control_world_from_root;
        self.robot.q.as_mut_slice().copy_from_slice(q);
        self.robot.v.as_mut_slice().copy_from_slice(joint_velocity);
        self.joint_velocity.copy_from_slice(joint_velocity);
        self.program
            .model
            .forward_kinematics(&self.robot, &mut self.cache)
            .map_err(value_error)?;
        self.program
            .model
            .floating_com_jacobian_into(
                &self.cache,
                &mut self.capture_dynamics,
                &mut self.center_of_mass_jacobian,
            )
            .map_err(value_error)?;
        for row in 0..2 {
            self.program
                .model
                .floating_point_jacobian_into(
                    &self.cache,
                    self.support_frames[row],
                    Vec3::zeros(),
                    &mut self.wheel_jacobians[row],
                )
                .map_err(value_error)?;
        }
        let generalized_velocity = |jacobian: &DMatrix<f64>, axis: usize| {
            let mut velocity = 0.0;
            for column in 0..6 {
                velocity += jacobian[(axis, column)] * root_twist_world[column];
            }
            for coordinate in 0..dof {
                velocity += jacobian[(axis, coordinate + 6)] * joint_velocity[coordinate];
            }
            velocity
        };
        let center_of_mass_velocity = generalized_velocity(&self.center_of_mass_jacobian, 0);
        let wheel_positions = self.support_frames.map(|frame| {
            self.cache.world_from_body[frame.0]
                .transform_point(&Point3::origin())
                .coords
        });
        let ground_position = 0.5 * (wheel_positions[0].x + wheel_positions[1].x);
        let ground_y = 0.5 * (wheel_positions[0].y + wheel_positions[1].y);
        let ground_velocity = 0.5
            * (generalized_velocity(&self.wheel_jacobians[0], 0)
                + generalized_velocity(&self.wheel_jacobians[1], 0));
        let station_point_control_world = control_world_from_odom.transform_point(&Point3::new(
            station_ground_position_odom_m,
            0.0,
            0.0,
        ));
        let physical_virtual_pitch = (self.cache.center_of_mass_world.x - ground_position)
            .atan2((self.cache.center_of_mass_world.z - ground_height_control_world_m).max(0.05));
        let capture = self
            .balancer
            .emit_capture_accelerations(
                timestep_seconds,
                station_point_control_world.x,
                ground_position,
                ground_velocity,
                self.cache.center_of_mass_world.x,
                center_of_mass_velocity,
                self.cache.center_of_mass_world.z - ground_height_control_world_m,
                &self.joint_velocity,
                self.capture_config,
                &mut self.capture_state,
                wheel_acceleration,
            )
            .ok_or_else(|| PyValueError::new_err("rooted capture reference is invalid"))?;
        self.external_frames
            .set(ExternalFrameSlotId(0), control_world_from_odom);
        self.external_frames.set(
            ExternalFrameSlotId(1),
            control_world_from_odom * map_from_odom.inverse(),
        );
        self.atlas
            .evaluate_into(&self.cache, &self.external_frames, &mut self.atlas_snapshot)
            .map_err(value_error)?;
        let ground_control_world =
            Point3::new(ground_position, ground_y, ground_height_control_world_m);
        let ground_odom = control_world_from_odom
            .inverse()
            .transform_point(&ground_control_world);
        let control_world_from_map = control_world_from_odom * map_from_odom.inverse();
        let ground_map = control_world_from_map
            .inverse()
            .transform_point(&ground_control_world);
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "rooted Upkie capture step allocated inside the Rust hot path",
            ));
        }
        diagnostics_out.copy_from_slice(&[
            physical_virtual_pitch,
            capture.capture_position_control_world_m,
            capture.capture_error_m,
            capture.capture_pressure,
            capture.station_error_m,
            capture.station_authority,
            ground_position,
            ground_velocity,
            station_point_control_world.x,
            capture.commanded_ground_velocity_mps,
            ground_odom.x,
            ground_map.x,
            self.cache.center_of_mass_world.x,
            center_of_mass_velocity,
            capture.natural_frequency_rad_s,
            capture.used_center_of_mass_height_m,
        ]);
        Ok(())
    }

    /// Rooted planar capture with differential-wheel heading authority. The
    /// returned heading defines the explicit world contact basis consumed by
    /// the floating WBC; Python remains responsible only for array transfer.
    #[allow(clippy::too_many_arguments)]
    fn step_planar_capture_from_state(
        &mut self,
        timestep_seconds: f64,
        station_ground_position_odom_m: f64,
        ground_height_control_world_m: f64,
        control_world_from_odom_translation: PyReadonlyArray1<'_, f64>,
        control_world_from_odom_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        map_from_odom_translation: PyReadonlyArray1<'_, f64>,
        map_from_odom_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        root_twist_world: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        joint_velocity: PyReadonlyArray1<'_, f64>,
        mut wheel_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut diagnostics_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<()> {
        let control_world_from_odom_translation = control_world_from_odom_translation.as_slice()?;
        let control_world_from_odom_quaternion_wxyz =
            control_world_from_odom_quaternion_wxyz.as_slice()?;
        let map_from_odom_translation = map_from_odom_translation.as_slice()?;
        let map_from_odom_quaternion_wxyz = map_from_odom_quaternion_wxyz.as_slice()?;
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let root_twist_world = root_twist_world.as_slice()?;
        let q = q.as_slice()?;
        let joint_velocity = joint_velocity.as_slice()?;
        let wheel_acceleration_out = wheel_acceleration_out.as_slice_mut()?;
        let diagnostics_out = diagnostics_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let valid_shapes = control_world_from_odom_translation.len() == 3
            && control_world_from_odom_quaternion_wxyz.len() == 4
            && map_from_odom_translation.len() == 3
            && map_from_odom_quaternion_wxyz.len() == 4
            && root_position.len() == 3
            && root_quaternion_wxyz.len() == 4
            && root_twist_world.len() == 6
            && q.len() == dof
            && joint_velocity.len() == dof
            && wheel_acceleration_out.len() == 2
            && diagnostics_out.len() == 44;
        if !valid_shapes {
            return Err(PyValueError::new_err(format!(
                "planar capture expects translations[3], quaternions[4], root twist[6], q/v[{dof}], wheel_out[2], and diagnostics_out[44]"
            )));
        }
        if !timestep_seconds.is_finite()
            || timestep_seconds <= 0.0
            || !station_ground_position_odom_m.is_finite()
            || !ground_height_control_world_m.is_finite()
            || control_world_from_odom_translation
                .iter()
                .chain(control_world_from_odom_quaternion_wxyz)
                .chain(map_from_odom_translation)
                .chain(map_from_odom_quaternion_wxyz)
                .chain(root_position)
                .chain(root_quaternion_wxyz)
                .chain(root_twist_world)
                .chain(q)
                .chain(joint_velocity)
                .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "planar capture state must be finite with a positive timestep",
            ));
        }
        let transform = |translation: &[f64], quaternion: &[f64]| -> PyResult<Transform3> {
            let rotation = UnitQuaternion::try_new(
                nalgebra::Quaternion::new(
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                ),
                1.0e-12,
            )
            .ok_or_else(|| PyValueError::new_err("planar capture quaternion is degenerate"))?;
            Ok(Transform3::from_parts(
                Translation3::new(translation[0], translation[1], translation[2]),
                rotation,
            ))
        };
        let control_world_from_odom = transform(
            control_world_from_odom_translation,
            control_world_from_odom_quaternion_wxyz,
        )?;
        let map_from_odom = transform(map_from_odom_translation, map_from_odom_quaternion_wxyz)?;
        let control_world_from_root = transform(root_position, root_quaternion_wxyz)?;
        let wheel_acceleration: &mut [f64; 2] = wheel_acceleration_out
            .try_into()
            .expect("wheel acceleration length was validated");

        let allocation_before = allocation_snapshot();
        self.robot.control_world_from_root = control_world_from_root;
        self.robot.q.as_mut_slice().copy_from_slice(q);
        self.robot.v.as_mut_slice().copy_from_slice(joint_velocity);
        self.joint_velocity.copy_from_slice(joint_velocity);
        self.program
            .model
            .forward_kinematics(&self.robot, &mut self.cache)
            .map_err(value_error)?;
        self.program
            .model
            .floating_com_jacobian_into(
                &self.cache,
                &mut self.capture_dynamics,
                &mut self.center_of_mass_jacobian,
            )
            .map_err(value_error)?;
        for row in 0..2 {
            self.program
                .model
                .floating_point_jacobian_into(
                    &self.cache,
                    self.support_frames[row],
                    Vec3::zeros(),
                    &mut self.wheel_jacobians[row],
                )
                .map_err(value_error)?;
        }
        let generalized_velocity = |jacobian: &DMatrix<f64>, axis: usize| {
            let root = (0..6)
                .map(|column| jacobian[(axis, column)] * root_twist_world[column])
                .sum::<f64>();
            root + (0..dof)
                .map(|coordinate| jacobian[(axis, coordinate + 6)] * joint_velocity[coordinate])
                .sum::<f64>()
        };
        let center_of_mass_velocity = Vec3::new(
            generalized_velocity(&self.center_of_mass_jacobian, 0),
            generalized_velocity(&self.center_of_mass_jacobian, 1),
            generalized_velocity(&self.center_of_mass_jacobian, 2),
        );
        let wheel_positions = self.support_frames.map(|frame| {
            self.cache.world_from_body[frame.0]
                .transform_point(&Point3::origin())
                .coords
        });
        let wheel_center = 0.5 * (wheel_positions[0] + wheel_positions[1]);
        let wheel_velocity = Vec3::new(
            0.5 * (generalized_velocity(&self.wheel_jacobians[0], 0)
                + generalized_velocity(&self.wheel_jacobians[1], 0)),
            0.5 * (generalized_velocity(&self.wheel_jacobians[0], 1)
                + generalized_velocity(&self.wheel_jacobians[1], 1)),
            0.5 * (generalized_velocity(&self.wheel_jacobians[0], 2)
                + generalized_velocity(&self.wheel_jacobians[1], 2)),
        );
        let station_point_control_world = control_world_from_odom.transform_point(&Point3::new(
            station_ground_position_odom_m,
            0.0,
            0.0,
        ));
        let heading_x = self
            .robot
            .control_world_from_root
            .rotation
            .transform_vector(&Vec3::x());
        let heading_norm = heading_x.xy().norm();
        if heading_norm <= 1.0e-9 {
            return Err(PyValueError::new_err(
                "planar capture root heading is vertical",
            ));
        }
        let tangent_x = Vec3::new(heading_x.x / heading_norm, heading_x.y / heading_norm, 0.0);
        let tangent_y = Vec3::new(-tangent_x.y, tangent_x.x, 0.0);
        let physical_virtual_pitch = ((self.cache.center_of_mass_world - wheel_center)
            .dot(&tangent_x))
        .atan2((self.cache.center_of_mass_world.z - ground_height_control_world_m).max(0.05));
        let capture = self
            .balancer
            .emit_planar_capture_accelerations(
                timestep_seconds,
                station_point_control_world.coords,
                wheel_positions[0],
                wheel_positions[1],
                wheel_velocity,
                self.cache.center_of_mass_world,
                center_of_mass_velocity,
                self.cache.center_of_mass_world.z - ground_height_control_world_m,
                heading_x,
                root_twist_world[2],
                &self.joint_velocity,
                self.planar_config,
                &mut self.planar_state,
                wheel_acceleration,
            )
            .ok_or_else(|| PyValueError::new_err("planar capture reference is invalid"))?;
        let (measured_roll, _, _) = self.robot.control_world_from_root.rotation.euler_angles();
        let measured_angular_velocity = Vec3::new(
            root_twist_world[0],
            root_twist_world[1],
            root_twist_world[2],
        );
        let viability = step_upkie_lateral_viability(
            timestep_seconds,
            (self.cache.center_of_mass_world - wheel_center).dot(&tangent_y),
            center_of_mass_velocity.dot(&tangent_y),
            self.cache.center_of_mass_world.z - ground_height_control_world_m,
            0.5 * capture.wheel_track_m,
            measured_roll,
            measured_angular_velocity.dot(&tangent_x),
            self.lateral_viability_config,
            &mut self.lateral_viability_state,
        )
        .ok_or_else(|| PyValueError::new_err("lateral viability reference is invalid"))?;
        self.external_frames
            .set(ExternalFrameSlotId(0), control_world_from_odom);
        self.external_frames.set(
            ExternalFrameSlotId(1),
            control_world_from_odom * map_from_odom.inverse(),
        );
        self.atlas
            .evaluate_into(&self.cache, &self.external_frames, &mut self.atlas_snapshot)
            .map_err(value_error)?;
        let ground_odom = control_world_from_odom
            .inverse()
            .transform_point(&Point3::from(wheel_center));
        let control_world_from_map = control_world_from_odom * map_from_odom.inverse();
        let ground_map = control_world_from_map
            .inverse()
            .transform_point(&Point3::from(wheel_center));
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "planar Upkie capture step allocated inside the Rust hot path",
            ));
        }
        let sagittal = capture.sagittal;
        let planar_station_authority = sagittal
            .station_authority
            .min(1.0 - capture.lateral_capture_pressure);
        let ground_longitudinal = wheel_center.dot(&tangent_x);
        diagnostics_out.copy_from_slice(&[
            physical_virtual_pitch,
            ground_longitudinal + sagittal.capture_position_control_world_m,
            sagittal.capture_error_m,
            capture.planar_capture_pressure,
            sagittal.station_error_m,
            planar_station_authority,
            ground_longitudinal,
            wheel_velocity.dot(&tangent_x),
            station_point_control_world.coords.dot(&tangent_x),
            sagittal.commanded_ground_velocity_mps,
            ground_odom.x,
            ground_map.x,
            self.cache.center_of_mass_world.dot(&tangent_x),
            center_of_mass_velocity.dot(&tangent_x),
            sagittal.natural_frequency_rad_s,
            sagittal.used_center_of_mass_height_m,
            capture.heading_world_rad,
            capture.longitudinal_capture_error_m,
            capture.lateral_capture_error_m,
            capture.planar_capture_error_m,
            capture.lateral_capture_pressure,
            capture.planar_capture_pressure,
            capture.desired_heading_error_rad,
            capture.target_yaw_rate_rad_s,
            capture.commanded_yaw_rate_rad_s,
            capture.commanded_yaw_acceleration_rad_s2,
            capture.wheel_track_m,
            u8::from(capture.steering_was_saturated) as f64,
            capture.steering_direction,
            capture.lateral_support_margin_m,
            viability.lateral_dcm_m,
            viability.support_limit_m,
            viability.viability_margin_m,
            viability.unconstrained_zmp_m,
            viability.commanded_zmp_m,
            viability.requested_lateral_acceleration_m_s2,
            viability.commanded_lateral_acceleration_m_s2,
            viability.target_bank_angle_rad,
            viability.commanded_bank_angle_rad,
            viability.commanded_roll_acceleration_rad_s2,
            viability.activation_pressure,
            u8::from(viability.zmp_was_saturated) as f64,
            u8::from(viability.acceleration_was_saturated) as f64,
            u8::from(viability.bank_was_saturated) as f64,
        ]);
        Ok(())
    }
}

/// Policy- and physics-free end-to-end CPU transaction used by Python evals.
/// Observations are supplied for every tick; Rust owns WBC, actuation mapping,
/// exact command splicing, segment validation, and dense block sampling.
#[pyclass(unsendable)]
struct DynamicAdvanceSession {
    program: MotionProgram,
    controller: FloatingDynamicController,
    state_in: FloatingDynamicControllerState,
    state_out: FloatingDynamicControllerState,
    output: FloatingDynamicControllerOutput,
    scratch: FloatingDynamicControllerScratch,
    observed: RobotState,
    desired_generalized_acceleration: DVector<f64>,
    generalized_acceleration_bounds: VelocityBounds,
    torque_bounds: VelocityBounds,
    contacts: Vec<ContactSpec>,
}

#[pymethods]
impl DynamicAdvanceSession {
    #[new]
    #[pyo3(signature = (
        urdf_path,
        contact_frame_names,
        friction_coefficient=0.8,
        maximum_generalized_acceleration=100.0,
        maximum_generalized_effort=1000.0,
        self_collision_clearance=None,
        certify_collision_between_samples=false,
        collision_max_subdivision_depth=0,
        world_sdf_values_zyx=None,
        world_grid_origin_world=None,
        world_grid_spacing=None,
        world_probe_body_names=None,
        world_outside_policy=0,
        world_collision_clearance=None,
        certify_world_collision_between_samples=false,
        world_collision_max_subdivision_depth=0,
        root_prediction_error_growth=None,
        world_scene_epoch=0,
        world_scene_source_time_ns=None,
        world_scene_valid_from_ns=None,
        world_scene_valid_until_ns=None,
        expected_world_scene_epoch=None,
        maximum_world_scene_age_ns=None,
        require_world_scene_horizon_validity=false,
        command_tracking_position_contingency=None,
        command_tracking_position_reject=None,
        command_tracking_velocity_contingency=None,
        command_tracking_velocity_reject=None,
        maximum_robot_observation_age_ns=None,
        maximum_robot_observation_sync_uncertainty_ns=None
    ))]
    fn new(
        urdf_path: &str,
        contact_frame_names: Vec<String>,
        friction_coefficient: f64,
        maximum_generalized_acceleration: f64,
        maximum_generalized_effort: f64,
        self_collision_clearance: Option<f64>,
        certify_collision_between_samples: bool,
        collision_max_subdivision_depth: u8,
        world_sdf_values_zyx: Option<PyReadonlyArray3<'_, f64>>,
        world_grid_origin_world: Option<PyReadonlyArray1<'_, f64>>,
        world_grid_spacing: Option<PyReadonlyArray1<'_, f64>>,
        world_probe_body_names: Option<Vec<String>>,
        world_outside_policy: u8,
        world_collision_clearance: Option<f64>,
        certify_world_collision_between_samples: bool,
        world_collision_max_subdivision_depth: u8,
        root_prediction_error_growth: Option<PyReadonlyArray1<'_, f64>>,
        world_scene_epoch: u64,
        world_scene_source_time_ns: Option<i64>,
        world_scene_valid_from_ns: Option<i64>,
        world_scene_valid_until_ns: Option<i64>,
        expected_world_scene_epoch: Option<u64>,
        maximum_world_scene_age_ns: Option<i64>,
        require_world_scene_horizon_validity: bool,
        command_tracking_position_contingency: Option<f64>,
        command_tracking_position_reject: Option<f64>,
        command_tracking_velocity_contingency: Option<f64>,
        command_tracking_velocity_reject: Option<f64>,
        maximum_robot_observation_age_ns: Option<i64>,
        maximum_robot_observation_sync_uncertainty_ns: Option<i64>,
    ) -> PyResult<Self> {
        if contact_frame_names.len() > 16
            || !friction_coefficient.is_finite()
            || friction_coefficient < 0.0
            || !maximum_generalized_acceleration.is_finite()
            || maximum_generalized_acceleration <= 0.0
            || !maximum_generalized_effort.is_finite()
            || maximum_generalized_effort <= 0.0
        {
            return Err(PyValueError::new_err(
                "dynamic advance configuration is invalid",
            ));
        }
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let outside_policy = match world_outside_policy {
            0 => SdfOutsidePolicy::Reject,
            1 => SdfOutsidePolicy::OccupiedBoundary,
            _ => {
                return Err(PyValueError::new_err(
                    "world_outside_policy must be 0 (reject) or 1 (occupied boundary)",
                ));
            }
        };
        let world_collision = match (
            world_sdf_values_zyx,
            world_grid_origin_world,
            world_grid_spacing,
        ) {
            (Some(values), Some(origin), Some(spacing)) => {
                let values = values.as_array();
                let origin = origin.as_slice()?;
                let spacing = spacing.as_slice()?;
                let shape = values.shape();
                if origin.len() != 3
                    || spacing.len() != 3
                    || shape.iter().any(|dimension| *dimension < 2)
                {
                    return Err(PyValueError::new_err("world SDF layout is invalid"));
                }
                let field = DenseSdfGrid::new(
                    bonesaw_core::Transform3::from_parts(
                        Translation3::new(origin[0], origin[1], origin[2]),
                        UnitQuaternion::identity(),
                    ),
                    [shape[2], shape[1], shape[0]],
                    Vec3::new(spacing[0], spacing[1], spacing[2]),
                    values.iter().copied().collect(),
                    outside_policy,
                )
                .map_err(value_error)?;
                let compiled = CompiledWorldCollisionModel::compile(&program.model, field)
                    .map_err(value_error)?;
                if let Some(body_names) = world_probe_body_names.as_ref() {
                    if body_names.is_empty() {
                        return Err(PyValueError::new_err(
                            "world_probe_body_names must not be empty when supplied",
                        ));
                    }
                    let mut body_ids = Vec::with_capacity(body_names.len());
                    for body_name in body_names {
                        let body = program.model.body_id(body_name).ok_or_else(|| {
                            PyValueError::new_err(format!("unknown world probe body {body_name}"))
                        })?;
                        if !body_ids.contains(&body) {
                            body_ids.push(body);
                        }
                    }
                    let CompiledWorldCollisionModel {
                        field,
                        probes,
                        unsupported_shape_count,
                        ..
                    } = compiled;
                    let probes = probes
                        .into_iter()
                        .filter(|probe| body_ids.contains(&probe.body))
                        .collect::<Vec<_>>();
                    if probes.is_empty() {
                        return Err(PyValueError::new_err(
                            "selected world probe bodies have no supported collision geometry",
                        ));
                    }
                    Some(
                        CompiledWorldCollisionModel::new(
                            &program.model,
                            field,
                            probes,
                            unsupported_shape_count,
                        )
                        .map_err(value_error)?,
                    )
                } else {
                    Some(compiled)
                }
            }
            (None, None, None) => None,
            _ => {
                return Err(PyValueError::new_err(
                    "world SDF values, origin, and spacing must be supplied together",
                ));
            }
        };
        let world_collision = world_collision
            .map(|world| {
                world.with_scene_stamp(WorldSceneStamp {
                    scene_epoch: world_scene_epoch,
                    source_time_ns: world_scene_source_time_ns.unwrap_or(i64::MIN),
                    valid_from_ns: world_scene_valid_from_ns.unwrap_or(i64::MIN),
                    valid_until_ns: world_scene_valid_until_ns.unwrap_or(i64::MAX),
                })
            })
            .transpose()
            .map_err(value_error)?;
        if world_probe_body_names.is_some() && world_collision.is_none() {
            return Err(PyValueError::new_err(
                "world_probe_body_names requires a world SDF",
            ));
        }
        if world_collision_clearance.is_some() != world_collision.is_some() {
            return Err(PyValueError::new_err(
                "world collision clearance and world SDF must be enabled together",
            ));
        }
        let root_prediction_error_growth = if let Some(values) = root_prediction_error_growth {
            let values = values.as_slice()?;
            if values.len() != 6 {
                return Err(PyValueError::new_err(
                    "root_prediction_error_growth must contain 6 nonnegative values",
                ));
            }
            RootPredictionErrorGrowth {
                initial_translation_radius_m: values[0],
                translation_velocity_error_bound_mps: values[1],
                translation_acceleration_error_bound_mps2: values[2],
                initial_rotation_radius_rad: values[3],
                angular_velocity_error_bound_radps: values[4],
                angular_acceleration_error_bound_radps2: values[5],
            }
        } else {
            RootPredictionErrorGrowth::default()
        };
        if !root_prediction_error_growth.is_valid() {
            return Err(PyValueError::new_err(
                "root_prediction_error_growth must contain 6 finite nonnegative values",
            ));
        }
        let validation = DynamicTrajectoryValidationConfig {
            self_collision_clearance,
            collision_continuity: if certify_collision_between_samples {
                CollisionContinuityPolicy::ConservativeRateBound
            } else {
                CollisionContinuityPolicy::GridOnly
            },
            collision_max_subdivision_depth,
            world_collision_clearance,
            world_collision_continuity: if certify_world_collision_between_samples {
                CollisionContinuityPolicy::ConservativeRateBound
            } else {
                CollisionContinuityPolicy::GridOnly
            },
            world_collision_max_subdivision_depth,
            root_prediction_error_growth,
            expected_world_scene_epoch,
            maximum_world_scene_age_ns,
            require_world_scene_horizon_validity,
            command_tracking_limits: CommandTrackingLimits {
                position_contingency: command_tracking_position_contingency,
                position_reject: command_tracking_position_reject,
                velocity_contingency: command_tracking_velocity_contingency,
                velocity_reject: command_tracking_velocity_reject,
            },
            robot_observation_limits: RobotObservationLimits {
                maximum_age_ns: maximum_robot_observation_age_ns,
                maximum_synchronization_uncertainty_ns:
                    maximum_robot_observation_sync_uncertainty_ns,
            },
            ..DynamicTrajectoryValidationConfig::default()
        };
        let controller = if let Some(world) = world_collision.as_ref() {
            FloatingDynamicController::from_program_with_world_collision(
                &program,
                DynamicWbcConfig::default(),
                validation,
                world.clone(),
            )
        } else {
            FloatingDynamicController::from_program_with_validation(
                &program,
                DynamicWbcConfig::default(),
                validation,
            )
        }
        .map_err(value_error)?;
        let observed = RobotState::zeros(&program.model);
        let state_in = FloatingDynamicControllerState::for_program(&observed, &program)
            .map_err(value_error)?;
        let state_out = state_in.clone();
        let maximum_contacts = contact_frame_names.len();
        let output = if let Some(world) = world_collision.as_ref() {
            FloatingDynamicControllerOutput::workspace_with_world_collision(
                &program,
                maximum_contacts,
                world,
            )
        } else {
            FloatingDynamicControllerOutput::workspace(&program, maximum_contacts)
        }
        .map_err(value_error)?;
        let scratch = if let Some(world) = world_collision.as_ref() {
            FloatingDynamicControllerScratch::for_program_with_world_collision(
                &program,
                maximum_contacts,
                world,
            )
        } else {
            FloatingDynamicControllerScratch::for_program(&program, maximum_contacts)
        };
        let generalized_dof = program.model.dof + 6;
        let desired_generalized_acceleration = DVector::zeros(generalized_dof);
        let generalized_acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -maximum_generalized_acceleration),
            upper: DVector::from_element(generalized_dof, maximum_generalized_acceleration),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(program.model.dof, -maximum_generalized_effort),
            upper: DVector::from_element(program.model.dof, maximum_generalized_effort),
        };
        let total_weight = program
            .model
            .bodies
            .iter()
            .map(|body| body.mass)
            .sum::<f64>()
            * 9.81;
        let nominal_normal_force = if maximum_contacts == 0 {
            0.0
        } else {
            total_weight / maximum_contacts as f64
        };
        let mut contacts = Vec::with_capacity(maximum_contacts);
        for (slot, frame_name) in contact_frame_names.iter().enumerate() {
            let frame = program.model.frame_id(frame_name).ok_or_else(|| {
                PyValueError::new_err(format!("unknown contact frame {frame_name}"))
            })?;
            contacts.push(ContactSpec::horizontal(
                slot as u32 + 1,
                frame,
                Vec3::zeros(),
                ContactMode::RollingPoint,
                friction_coefficient,
                total_weight.max(1.0),
                nominal_normal_force,
            ));
        }
        Ok(Self {
            program,
            controller,
            state_in,
            state_out,
            output,
            scratch,
            observed,
            desired_generalized_acceleration,
            generalized_acceleration_bounds,
            torque_bounds,
            contacts,
        })
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn generalized_dof(&self) -> usize {
        self.program.model.dof + 6
    }

    #[getter]
    fn actuator_count(&self) -> usize {
        self.program.actuation.actuators.len()
    }

    #[getter]
    fn samples_per_tick(&self) -> usize {
        (self.program.timing.control_horizon_ns / self.program.timing.sample_period_ns) as usize
    }

    /// Diagnostics for the most recent primary and contingency continuity
    /// queries. Python owns retention/reporting; the timed Rust transaction
    /// writes only fixed scalar fields.
    fn continuity_diagnostics(&self) -> (usize, usize, usize, u8, usize, usize, usize, u8) {
        (
            self.output.primary_continuity_leaf_interval_count,
            self.output.primary_collision_refinement_pair_samples,
            self.output.primary_continuity_unresolved_interval_count,
            self.output.primary_continuity_maximum_subdivision_depth,
            self.output.contingency_continuity_leaf_interval_count,
            self.output.contingency_collision_refinement_pair_samples,
            self.output.contingency_continuity_unresolved_interval_count,
            self.output.contingency_continuity_maximum_subdivision_depth,
        )
    }

    /// Reset caller-visible command memory for an independent experiment.
    /// This is setup work outside the measured transaction and does not alter
    /// the immutable controller or program.
    fn reset_command_state(
        &mut self,
        initial_q: PyReadonlyArray1<'_, f64>,
        initial_v: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        let q = initial_q.as_slice()?;
        let v = initial_v.as_slice()?;
        if q.len() != self.dof()
            || v.len() != self.dof()
            || !q.iter().chain(v.iter()).all(|value| value.is_finite())
        {
            return Err(PyValueError::new_err(
                "dynamic command-state reset shape or value is invalid",
            ));
        }
        self.observed.q.as_mut_slice().copy_from_slice(q);
        self.observed.v.as_mut_slice().copy_from_slice(v);
        self.state_in = FloatingDynamicControllerState::for_program(&self.observed, &self.program)
            .map_err(value_error)?;
        self.state_out = self.state_in.clone();
        Ok(())
    }

    /// Evidence from the most recent transaction. This read-only adapter is
    /// outside the timed control call; the evidence itself is computed in Rust
    /// without allocation.
    fn command_tracking_evidence(&self) -> (f64, f64, i64, i64, f64, f64, f64, f64, u8) {
        let evidence = self.output.command_tracking;
        (
            evidence.maximum_position_error,
            evidence.maximum_velocity_error,
            evidence
                .limiting_position_actuator
                .map_or(-1, |actuator| actuator as i64),
            evidence
                .limiting_velocity_actuator
                .map_or(-1, |actuator| actuator as i64),
            evidence.position_contingency_headroom,
            evidence.position_reject_headroom,
            evidence.velocity_contingency_headroom,
            evidence.velocity_reject_headroom,
            command_tracking_action_code(evidence.action),
        )
    }

    /// Execute one explicitly stamped observation transaction. The nested
    /// Python tuples are created after the measured allocation-free Rust call.
    /// Decision: `(status, selection, flags, primary_valid, contingency_valid)`.
    /// Evidence: `(source_time, mapped_time, sequence, source_id, sync_error,
    /// age, age_headroom, sync_headroom, causal, age_valid, sync_valid)`.
    /// Timing: `(elapsed_ns, allocation_calls, allocated_bytes)`.
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    fn advance_observation(
        &mut self,
        py: Python<'_>,
        tick_time_ns: i64,
        source_time_ns: i64,
        mapped_time_ns: i64,
        source_sequence: u64,
        source_id: u64,
        synchronization_uncertainty_ns: i64,
        observed_q: PyReadonlyArray1<'_, f64>,
        observed_v: PyReadonlyArray1<'_, f64>,
        root_twist_world: PyReadonlyArray1<'_, f64>,
        desired_generalized_acceleration: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<(
        (u8, u8, u32, bool, bool),
        (i64, i64, u64, u64, i64, i64, i64, i64, bool, bool, bool),
        (u64, u64, u64),
    )> {
        let observed_q = observed_q.as_slice()?;
        let observed_v = observed_v.as_slice()?;
        let root_twist_world = root_twist_world.as_slice()?;
        let desired = desired_generalized_acceleration.as_slice()?;
        if observed_q.len() != self.dof()
            || observed_v.len() != self.dof()
            || root_twist_world.len() != 6
            || desired.len() != self.generalized_dof()
            || !observed_q
                .iter()
                .chain(observed_v.iter())
                .chain(root_twist_world.iter())
                .chain(desired.iter())
                .all(|value| value.is_finite())
        {
            return Err(PyValueError::new_err(
                "stamped observation shape or value is invalid",
            ));
        }
        self.observed.q.as_mut_slice().copy_from_slice(observed_q);
        self.observed.v.as_mut_slice().copy_from_slice(observed_v);
        self.desired_generalized_acceleration
            .as_mut_slice()
            .copy_from_slice(desired);
        let root_twist = Motion6(nalgebra::SVector::<f64, 6>::from_column_slice(
            root_twist_world,
        ));
        let stamp = RobotObservationStamp {
            source_time_ns,
            mapped_time_ns,
            source_sequence,
            source_id,
            synchronization_uncertainty_ns,
        };
        let timing = py.detach(|| {
            let before = allocation_snapshot();
            let started = Instant::now();
            self.controller
                .advance_into(
                    FloatingDynamicControllerInput {
                        tick_time_ns,
                        program_epoch: self.program.header.program_epoch,
                        observation_stamp: stamp,
                        wbc: FloatingDynamicWbcInput {
                            state: &self.observed,
                            root_twist_world: root_twist,
                            desired_generalized_acceleration: &self
                                .desired_generalized_acceleration,
                            task_priorities: FloatingTaskPriorities::default(),
                            task_weights: FloatingTaskWeights::default(),
                            joint_posture_weight: 1.0,
                            joint_acceleration_task: None,
                            center_of_mass_task: None,
                            centroidal_angular_momentum_task: None,
                            frame_angular_acceleration_tasks: &[],
                            point_acceleration_tasks: &[],
                            generalized_acceleration_bounds: &self.generalized_acceleration_bounds,
                            torque_bounds: &self.torque_bounds,
                            actuator_effort: None,
                            contacts: &self.contacts,
                            support_patches: &[],
                        },
                    },
                    &self.state_in,
                    &mut self.state_out,
                    &mut self.output,
                    &mut self.scratch,
                )
                .map_err(value_error)?;
            let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let after = allocation_snapshot();
            Ok::<_, PyErr>((
                elapsed_ns,
                after.0.saturating_sub(before.0),
                after.1.saturating_sub(before.1),
            ))
        })?;
        if self.output.selection != bonesaw_core::DynamicPlanSelection::Rejected {
            std::mem::swap(&mut self.state_in, &mut self.state_out);
        }
        let evidence = self.output.robot_observation;
        Ok((
            (
                dynamic_status_code(self.output.status),
                dynamic_selection_code(self.output.selection),
                self.output.admission_flags.bits(),
                self.output.primary_valid,
                self.output.contingency_valid,
            ),
            (
                evidence.stamp.source_time_ns,
                evidence.stamp.mapped_time_ns,
                evidence.stamp.source_sequence,
                evidence.stamp.source_id,
                evidence.stamp.synchronization_uncertainty_ns,
                evidence.age_ns,
                evidence.age_headroom_ns,
                evidence.synchronization_uncertainty_headroom_ns,
                evidence.causal,
                evidence.age_valid,
                evidence.synchronization_valid,
            ),
            timing,
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn run_trace(
        &mut self,
        py: Python<'_>,
        start_time_ns: i64,
        observed_q: PyReadonlyArray2<'_, f64>,
        observed_v: PyReadonlyArray2<'_, f64>,
        root_twist_world: PyReadonlyArray2<'_, f64>,
        desired_generalized_acceleration: PyReadonlyArray2<'_, f64>,
        mut sample_position_out: PyReadwriteArray3<'_, f64>,
        mut sample_velocity_out: PyReadwriteArray3<'_, f64>,
        mut sample_acceleration_out: PyReadwriteArray3<'_, f64>,
        mut generalized_acceleration_out: PyReadwriteArray2<'_, f64>,
        mut splice_position_out: PyReadwriteArray2<'_, f64>,
        mut splice_velocity_out: PyReadwriteArray2<'_, f64>,
        mut splice_acceleration_out: PyReadwriteArray2<'_, f64>,
        mut admitted_effort_out: PyReadwriteArray2<'_, f64>,
        mut selected_extrema_out: PyReadwriteArray2<'_, f64>,
        mut selected_position_headroom_out: PyReadwriteArray1<'_, f64>,
        mut admission_flags_out: PyReadwriteArray1<'_, u32>,
        mut primary_collision_distance_out: PyReadwriteArray1<'_, f64>,
        mut contingency_collision_distance_out: PyReadwriteArray1<'_, f64>,
        mut primary_continuous_clearance_out: PyReadwriteArray1<'_, f64>,
        mut contingency_continuous_clearance_out: PyReadwriteArray1<'_, f64>,
        mut primary_relative_speed_bound_out: PyReadwriteArray1<'_, f64>,
        mut contingency_relative_speed_bound_out: PyReadwriteArray1<'_, f64>,
        mut primary_collision_pair_out: PyReadwriteArray1<'_, i64>,
        mut primary_collision_time_ns_out: PyReadwriteArray1<'_, i64>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut selection_out: PyReadwriteArray1<'_, u8>,
        mut solve_status_out: PyReadwriteArray1<'_, u8>,
        mut primary_valid_out: PyReadwriteArray1<'_, u8>,
        mut contingency_valid_out: PyReadwriteArray1<'_, u8>,
        mut previous_plan_expired_out: PyReadwriteArray1<'_, u8>,
        mut dynamics_residual_out: PyReadwriteArray1<'_, f64>,
        mut contact_residual_out: PyReadwriteArray1<'_, f64>,
        mut command_divergence_out: PyReadwriteArray1<'_, f64>,
        mut step_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let observed_q = observed_q.as_array();
        let observed_v = observed_v.as_array();
        let root_twist_world = root_twist_world.as_array();
        let desired_generalized_acceleration = desired_generalized_acceleration.as_array();
        let mut sample_position_out = sample_position_out.as_array_mut();
        let mut sample_velocity_out = sample_velocity_out.as_array_mut();
        let mut sample_acceleration_out = sample_acceleration_out.as_array_mut();
        let mut generalized_acceleration_out = generalized_acceleration_out.as_array_mut();
        let mut splice_position_out = splice_position_out.as_array_mut();
        let mut splice_velocity_out = splice_velocity_out.as_array_mut();
        let mut splice_acceleration_out = splice_acceleration_out.as_array_mut();
        let mut admitted_effort_out = admitted_effort_out.as_array_mut();
        let mut selected_extrema_out = selected_extrema_out.as_array_mut();
        let selected_position_headroom_out = selected_position_headroom_out.as_slice_mut()?;
        let admission_flags_out = admission_flags_out.as_slice_mut()?;
        let primary_collision_distance_out = primary_collision_distance_out.as_slice_mut()?;
        let contingency_collision_distance_out =
            contingency_collision_distance_out.as_slice_mut()?;
        let primary_continuous_clearance_out = primary_continuous_clearance_out.as_slice_mut()?;
        let contingency_continuous_clearance_out =
            contingency_continuous_clearance_out.as_slice_mut()?;
        let primary_relative_speed_bound_out = primary_relative_speed_bound_out.as_slice_mut()?;
        let contingency_relative_speed_bound_out =
            contingency_relative_speed_bound_out.as_slice_mut()?;
        let primary_collision_pair_out = primary_collision_pair_out.as_slice_mut()?;
        let primary_collision_time_ns_out = primary_collision_time_ns_out.as_slice_mut()?;
        let status_out = status_out.as_slice_mut()?;
        let selection_out = selection_out.as_slice_mut()?;
        let solve_status_out = solve_status_out.as_slice_mut()?;
        let primary_valid_out = primary_valid_out.as_slice_mut()?;
        let contingency_valid_out = contingency_valid_out.as_slice_mut()?;
        let previous_plan_expired_out = previous_plan_expired_out.as_slice_mut()?;
        let dynamics_residual_out = dynamics_residual_out.as_slice_mut()?;
        let contact_residual_out = contact_residual_out.as_slice_mut()?;
        let command_divergence_out = command_divergence_out.as_slice_mut()?;
        let step_ns_out = step_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let ticks = observed_q.nrows();
        let dof = self.dof();
        let generalized_dof = self.generalized_dof();
        let actuators = self.actuator_count();
        let samples = self.samples_per_tick();
        if observed_q.shape() != [ticks, dof]
            || observed_v.shape() != [ticks, dof]
            || root_twist_world.shape() != [ticks, 6]
            || desired_generalized_acceleration.shape() != [ticks, generalized_dof]
            || sample_position_out.shape() != [ticks, samples, actuators]
            || sample_velocity_out.shape() != [ticks, samples, actuators]
            || sample_acceleration_out.shape() != [ticks, samples, actuators]
            || generalized_acceleration_out.shape() != [ticks, generalized_dof]
            || splice_position_out.shape() != [ticks, actuators]
            || splice_velocity_out.shape() != [ticks, actuators]
            || splice_acceleration_out.shape() != [ticks, actuators]
            || admitted_effort_out.shape() != [ticks, actuators]
            || selected_extrema_out.shape() != [ticks, 3]
            || selected_position_headroom_out.len() != ticks
            || admission_flags_out.len() != ticks
            || primary_collision_distance_out.len() != ticks
            || contingency_collision_distance_out.len() != ticks
            || primary_continuous_clearance_out.len() != ticks
            || contingency_continuous_clearance_out.len() != ticks
            || primary_relative_speed_bound_out.len() != ticks
            || contingency_relative_speed_bound_out.len() != ticks
            || primary_collision_pair_out.len() != ticks
            || primary_collision_time_ns_out.len() != ticks
            || status_out.len() != ticks
            || selection_out.len() != ticks
            || solve_status_out.len() != ticks
            || primary_valid_out.len() != ticks
            || contingency_valid_out.len() != ticks
            || previous_plan_expired_out.len() != ticks
            || dynamics_residual_out.len() != ticks
            || contact_residual_out.len() != ticks
            || command_divergence_out.len() != ticks
            || step_ns_out.len() != ticks
            || allocation_calls_out.len() != ticks
            || allocated_bytes_out.len() != ticks
        {
            return Err(PyValueError::new_err(
                "dynamic advance trace/output array shape mismatch",
            ));
        }

        for tick in 0..ticks {
            for coordinate in 0..dof {
                self.observed.q[coordinate] = observed_q[[tick, coordinate]];
                self.observed.v[coordinate] = observed_v[[tick, coordinate]];
            }
            for coordinate in 0..generalized_dof {
                self.desired_generalized_acceleration[coordinate] =
                    desired_generalized_acceleration[[tick, coordinate]];
            }
            let root_twist = Motion6(nalgebra::SVector::<f64, 6>::from_fn(|axis, _| {
                root_twist_world[[tick, axis]]
            }));
            let tick_time_ns = start_time_ns + tick as i64 * self.program.timing.control_horizon_ns;
            let (step_ns, allocation_calls, allocated_bytes) = py.detach(|| {
                let before = allocation_snapshot();
                let started = Instant::now();
                self.controller
                    .advance_into(
                        FloatingDynamicControllerInput {
                            tick_time_ns,
                            program_epoch: self.program.header.program_epoch,
                            observation_stamp: RobotObservationStamp::exact_at(
                                tick_time_ns,
                                1,
                                tick as u64 + 1,
                            ),
                            wbc: FloatingDynamicWbcInput {
                                state: &self.observed,
                                root_twist_world: root_twist,
                                desired_generalized_acceleration: &self
                                    .desired_generalized_acceleration,
                                task_priorities: FloatingTaskPriorities::default(),
                                task_weights: FloatingTaskWeights::default(),
                                joint_posture_weight: 1.0,
                                joint_acceleration_task: None,
                                center_of_mass_task: None,
                                centroidal_angular_momentum_task: None,
                                frame_angular_acceleration_tasks: &[],
                                point_acceleration_tasks: &[],
                                generalized_acceleration_bounds: &self
                                    .generalized_acceleration_bounds,
                                torque_bounds: &self.torque_bounds,
                                actuator_effort: None,
                                contacts: &self.contacts,
                                support_patches: &[],
                            },
                        },
                        &self.state_in,
                        &mut self.state_out,
                        &mut self.output,
                        &mut self.scratch,
                    )
                    .map_err(value_error)?;
                let step_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let after = allocation_snapshot();
                Ok::<_, PyErr>((
                    step_ns,
                    after.0.saturating_sub(before.0),
                    after.1.saturating_sub(before.1),
                ))
            })?;
            step_ns_out[tick] = step_ns;
            allocation_calls_out[tick] = allocation_calls;
            allocated_bytes_out[tick] = allocated_bytes;
            admission_flags_out[tick] = self.output.admission_flags.bits();
            primary_collision_distance_out[tick] =
                self.output.primary_collision.minimum_signed_distance;
            contingency_collision_distance_out[tick] =
                self.output.contingency_collision.minimum_signed_distance;
            primary_continuous_clearance_out[tick] =
                self.output.primary_continuous_clearance_lower_bound;
            contingency_continuous_clearance_out[tick] =
                self.output.contingency_continuous_clearance_lower_bound;
            primary_relative_speed_bound_out[tick] =
                self.output.primary_maximum_relative_speed_bound;
            contingency_relative_speed_bound_out[tick] =
                self.output.contingency_maximum_relative_speed_bound;
            primary_collision_pair_out[tick] = self
                .output
                .primary_collision
                .first_violation_pair_id
                .map_or(-1, i64::from);
            primary_collision_time_ns_out[tick] = self
                .output
                .primary_collision
                .first_violation_time_ns
                .unwrap_or(-1);
            status_out[tick] = dynamic_status_code(self.output.status);
            selection_out[tick] = dynamic_selection_code(self.output.selection);
            solve_status_out[tick] = solve_status_code(self.output.wbc.status);
            primary_valid_out[tick] = u8::from(self.output.primary_valid);
            contingency_valid_out[tick] = u8::from(self.output.contingency_valid);
            previous_plan_expired_out[tick] = u8::from(self.output.previous_plan_expired);
            dynamics_residual_out[tick] = self.output.wbc.dynamics_residual_linf;
            contact_residual_out[tick] = self.output.wbc.contact_acceleration_residual_linf;
            command_divergence_out[tick] = self.output.maximum_observed_command_position_divergence;
            for coordinate in 0..generalized_dof {
                generalized_acceleration_out[[tick, coordinate]] =
                    self.output.wbc.generalized_acceleration[coordinate];
            }
            for actuator in 0..actuators {
                splice_position_out[[tick, actuator]] = self.output.splice_position[actuator];
                splice_velocity_out[[tick, actuator]] = self.output.splice_velocity[actuator];
                splice_acceleration_out[[tick, actuator]] =
                    self.output.splice_acceleration[actuator];
                admitted_effort_out[[tick, actuator]] =
                    self.output.admitted_actuator_effort[actuator];
            }
            let selected_extrema =
                if self.output.selection == bonesaw_core::DynamicPlanSelection::Primary {
                    &self.output.primary_extrema
                } else {
                    &self.output.contingency_extrema
                };
            selected_extrema_out[[tick, 0]] = selected_extrema
                .max_abs_velocity
                .iter()
                .copied()
                .fold(0.0, f64::max);
            selected_extrema_out[[tick, 1]] = selected_extrema
                .max_abs_acceleration
                .iter()
                .copied()
                .fold(0.0, f64::max);
            selected_extrema_out[[tick, 2]] = selected_extrema
                .max_abs_jerk
                .iter()
                .copied()
                .fold(0.0, f64::max);
            selected_position_headroom_out[tick] =
                if self.output.selection == bonesaw_core::DynamicPlanSelection::Primary {
                    self.output.primary_minimum_joint_position_headroom
                } else {
                    self.output.contingency_minimum_joint_position_headroom
                };
            for sample in 0..samples {
                let position = self.output.samples.position(sample);
                let velocity = self.output.samples.velocity(sample);
                let acceleration = self.output.samples.acceleration(sample);
                for actuator in 0..actuators {
                    sample_position_out[[tick, sample, actuator]] =
                        position.map_or(f64::NAN, |values| values[actuator]);
                    sample_velocity_out[[tick, sample, actuator]] =
                        velocity.map_or(f64::NAN, |values| values[actuator]);
                    sample_acceleration_out[[tick, sample, actuator]] =
                        acceleration.map_or(f64::NAN, |values| values[actuator]);
                }
            }
            if self.output.selection != bonesaw_core::DynamicPlanSelection::Rejected {
                std::mem::swap(&mut self.state_in, &mut self.state_out);
            }
        }
        Ok(())
    }

    /// Fixed world-command evidence layout:
    /// `metrics[:, :] = [primary_distance, contingency_distance,
    /// primary_continuous_lower, contingency_continuous_lower,
    /// primary_rate_bound, contingency_rate_bound, max_abs_admitted_effort]`.
    /// `ids[:, :] = [primary_min_probe, primary_min_body, contingency_min_probe,
    /// contingency_min_body, primary_violation_probe, primary_violation_time,
    /// contingency_violation_probe, contingency_violation_time,
    /// primary_unknown_probe, primary_unknown_time, contingency_unknown_probe,
    /// contingency_unknown_time, primary_continuity_probe,
    /// contingency_continuity_probe, flags, status, selection, primary_valid,
    /// contingency_valid, solve_status, primary_source, contingency_source,
    /// scene_epoch, scene_validity, scene_source_time, scene_valid_from,
    /// scene_valid_until, tick_time, horizon_end]`.
    /// Missing IDs/times use `u64::MAX`; source codes are 0=trilinear,
    /// 1=occupied boundary, `u64::MAX`=none. `work[:, :]` is primary then
    /// contingency `[leaves, midpoint_probe_samples, unresolved, depth]`.
    /// `root_prediction[:, :]` stores primary then contingency
    /// `[end translation xyz, end rotation-vector xyz, end twist 6,
    /// maximum-absolute twist 6, deterministic error growth 6]`, where error
    /// growth is `[translation radius, translation velocity/acceleration
    /// error, rotation radius, angular velocity/acceleration error]`. These
    /// are state-local prediction witnesses, not executable root commands or
    /// probabilistic confidence claims.
    /// `timing[:, :]` is `[elapsed_ns, allocation_calls, allocated_bytes]`.
    #[allow(clippy::too_many_arguments)]
    fn run_world_trace(
        &mut self,
        py: Python<'_>,
        start_time_ns: i64,
        observed_q: PyReadonlyArray2<'_, f64>,
        observed_v: PyReadonlyArray2<'_, f64>,
        root_twist_world: PyReadonlyArray2<'_, f64>,
        desired_generalized_acceleration: PyReadonlyArray2<'_, f64>,
        mut generalized_acceleration_out: PyReadwriteArray2<'_, f64>,
        mut splice_position_out: PyReadwriteArray2<'_, f64>,
        mut splice_velocity_out: PyReadwriteArray2<'_, f64>,
        mut splice_acceleration_out: PyReadwriteArray2<'_, f64>,
        mut metrics_out: PyReadwriteArray2<'_, f64>,
        mut ids_out: PyReadwriteArray2<'_, u64>,
        mut work_out: PyReadwriteArray2<'_, u64>,
        mut root_prediction_out: PyReadwriteArray2<'_, f64>,
        mut timing_out: PyReadwriteArray2<'_, u64>,
    ) -> PyResult<()> {
        let observed_q = observed_q.as_array();
        let observed_v = observed_v.as_array();
        let root_twist_world = root_twist_world.as_array();
        let desired_generalized_acceleration = desired_generalized_acceleration.as_array();
        let mut generalized_acceleration_out = generalized_acceleration_out.as_array_mut();
        let mut splice_position_out = splice_position_out.as_array_mut();
        let mut splice_velocity_out = splice_velocity_out.as_array_mut();
        let mut splice_acceleration_out = splice_acceleration_out.as_array_mut();
        let mut metrics_out = metrics_out.as_array_mut();
        let mut ids_out = ids_out.as_array_mut();
        let mut work_out = work_out.as_array_mut();
        let mut root_prediction_out = root_prediction_out.as_array_mut();
        let mut timing_out = timing_out.as_array_mut();
        let ticks = observed_q.nrows();
        let dof = self.dof();
        let generalized_dof = self.generalized_dof();
        let actuators = self.actuator_count();
        if observed_q.shape() != [ticks, dof]
            || observed_v.shape() != [ticks, dof]
            || root_twist_world.shape() != [ticks, 6]
            || desired_generalized_acceleration.shape() != [ticks, generalized_dof]
            || generalized_acceleration_out.shape() != [ticks, generalized_dof]
            || splice_position_out.shape() != [ticks, actuators]
            || splice_velocity_out.shape() != [ticks, actuators]
            || splice_acceleration_out.shape() != [ticks, actuators]
            || metrics_out.shape() != [ticks, 9]
            || ids_out.shape() != [ticks, 29]
            || work_out.shape() != [ticks, 8]
            || root_prediction_out.shape() != [ticks, 48]
            || timing_out.shape() != [ticks, 3]
            || !observed_q
                .iter()
                .chain(observed_v.iter())
                .chain(root_twist_world.iter())
                .chain(desired_generalized_acceleration.iter())
                .all(|value| value.is_finite())
        {
            return Err(PyValueError::new_err(
                "world command trace/output array shape or value mismatch",
            ));
        }
        let option_id = |value: Option<u32>| value.map_or(u64::MAX, u64::from);
        let option_body =
            |value: Option<bonesaw_core::BodyId>| value.map_or(u64::MAX, |body| body.0 as u64);
        let option_time = |value: Option<i64>| value.map_or(u64::MAX, |time| time as u64);
        let source_code = |value: Option<SdfSampleSource>| match value {
            Some(SdfSampleSource::TrilinearGrid) => 0,
            Some(SdfSampleSource::OccupiedBoundary) => 1,
            None => u64::MAX,
        };
        let scene_validity_code = |value: WorldSceneValidity| match value {
            WorldSceneValidity::Valid => 0,
            WorldSceneValidity::EpochMismatch => 1,
            WorldSceneValidity::SourceFromFuture => 2,
            WorldSceneValidity::NotYetValid => 3,
            WorldSceneValidity::ExpiredAtTick => 4,
            WorldSceneValidity::HorizonExpired => 5,
            WorldSceneValidity::TooOld => 6,
            WorldSceneValidity::InvalidStamp => 7,
        };
        for tick in 0..ticks {
            for coordinate in 0..dof {
                self.observed.q[coordinate] = observed_q[[tick, coordinate]];
                self.observed.v[coordinate] = observed_v[[tick, coordinate]];
            }
            for coordinate in 0..generalized_dof {
                self.desired_generalized_acceleration[coordinate] =
                    desired_generalized_acceleration[[tick, coordinate]];
            }
            let root_twist = Motion6(nalgebra::SVector::<f64, 6>::from_fn(|axis, _| {
                root_twist_world[[tick, axis]]
            }));
            let tick_time_ns = start_time_ns + tick as i64 * self.program.timing.control_horizon_ns;
            let (elapsed_ns, allocation_calls, allocated_bytes) = py.detach(|| {
                let before = allocation_snapshot();
                let started = Instant::now();
                self.controller
                    .advance_into(
                        FloatingDynamicControllerInput {
                            tick_time_ns,
                            program_epoch: self.program.header.program_epoch,
                            observation_stamp: RobotObservationStamp::exact_at(
                                tick_time_ns,
                                1,
                                tick as u64 + 1,
                            ),
                            wbc: FloatingDynamicWbcInput {
                                state: &self.observed,
                                root_twist_world: root_twist,
                                desired_generalized_acceleration: &self
                                    .desired_generalized_acceleration,
                                task_priorities: FloatingTaskPriorities::default(),
                                task_weights: FloatingTaskWeights::default(),
                                joint_posture_weight: 1.0,
                                joint_acceleration_task: None,
                                center_of_mass_task: None,
                                centroidal_angular_momentum_task: None,
                                frame_angular_acceleration_tasks: &[],
                                point_acceleration_tasks: &[],
                                generalized_acceleration_bounds: &self
                                    .generalized_acceleration_bounds,
                                torque_bounds: &self.torque_bounds,
                                actuator_effort: None,
                                contacts: &self.contacts,
                                support_patches: &[],
                            },
                        },
                        &self.state_in,
                        &mut self.state_out,
                        &mut self.output,
                        &mut self.scratch,
                    )
                    .map_err(value_error)?;
                let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let after = allocation_snapshot();
                Ok::<_, PyErr>((
                    elapsed_ns,
                    after.0.saturating_sub(before.0),
                    after.1.saturating_sub(before.1),
                ))
            })?;
            let primary = self.output.primary_world_collision;
            let contingency = self.output.contingency_world_collision;
            let primary_continuity = self.output.primary_world_continuity;
            let contingency_continuity = self.output.contingency_world_continuity;
            metrics_out[[tick, 0]] = primary.minimum_signed_distance_m;
            metrics_out[[tick, 1]] = contingency.minimum_signed_distance_m;
            metrics_out[[tick, 2]] = primary_continuity.minimum_clearance_lower_bound_m;
            metrics_out[[tick, 3]] = contingency_continuity.minimum_clearance_lower_bound_m;
            metrics_out[[tick, 4]] = primary_continuity.limiting_distance_rate_bound_mps;
            metrics_out[[tick, 5]] = contingency_continuity.limiting_distance_rate_bound_mps;
            metrics_out[[tick, 6]] = self
                .output
                .admitted_actuator_effort
                .iter()
                .map(|effort| effort.abs())
                .fold(0.0, f64::max);
            metrics_out[[tick, 7]] = primary.maximum_prediction_clearance_erosion_m;
            metrics_out[[tick, 8]] = contingency.maximum_prediction_clearance_erosion_m;
            ids_out[[tick, 0]] = option_id(primary.minimum_probe_id);
            ids_out[[tick, 1]] = option_body(primary.minimum_body);
            ids_out[[tick, 2]] = option_id(contingency.minimum_probe_id);
            ids_out[[tick, 3]] = option_body(contingency.minimum_body);
            ids_out[[tick, 4]] = option_id(primary.first_violation_probe_id);
            ids_out[[tick, 5]] = option_time(primary.first_violation_time_ns);
            ids_out[[tick, 6]] = option_id(contingency.first_violation_probe_id);
            ids_out[[tick, 7]] = option_time(contingency.first_violation_time_ns);
            ids_out[[tick, 8]] = option_id(primary.first_unknown_probe_id);
            ids_out[[tick, 9]] = option_time(primary.first_unknown_time_ns);
            ids_out[[tick, 10]] = option_id(contingency.first_unknown_probe_id);
            ids_out[[tick, 11]] = option_time(contingency.first_unknown_time_ns);
            ids_out[[tick, 12]] = option_id(primary_continuity.limiting_probe_id);
            ids_out[[tick, 13]] = option_id(contingency_continuity.limiting_probe_id);
            ids_out[[tick, 14]] = u64::from(self.output.admission_flags.bits());
            ids_out[[tick, 15]] = u64::from(dynamic_status_code(self.output.status));
            ids_out[[tick, 16]] = u64::from(dynamic_selection_code(self.output.selection));
            ids_out[[tick, 17]] = u64::from(self.output.primary_valid);
            ids_out[[tick, 18]] = u64::from(self.output.contingency_valid);
            ids_out[[tick, 19]] = u64::from(solve_status_code(self.output.wbc.status));
            ids_out[[tick, 20]] = source_code(primary.minimum_field_source);
            ids_out[[tick, 21]] = source_code(contingency.minimum_field_source);
            let scene = self.output.world_scene;
            ids_out[[tick, 22]] = scene.stamp.scene_epoch;
            ids_out[[tick, 23]] = scene_validity_code(scene.validity);
            ids_out[[tick, 24]] = scene.stamp.source_time_ns as u64;
            ids_out[[tick, 25]] = scene.stamp.valid_from_ns as u64;
            ids_out[[tick, 26]] = scene.stamp.valid_until_ns as u64;
            ids_out[[tick, 27]] = scene.tick_time_ns as u64;
            ids_out[[tick, 28]] = scene.horizon_end_ns as u64;
            work_out[[tick, 0]] = primary_continuity.leaf_interval_count as u64;
            work_out[[tick, 1]] = primary_continuity.refinement_probe_samples_evaluated as u64;
            work_out[[tick, 2]] = primary_continuity.unresolved_interval_count as u64;
            work_out[[tick, 3]] = u64::from(primary_continuity.maximum_subdivision_depth_reached);
            work_out[[tick, 4]] = contingency_continuity.leaf_interval_count as u64;
            work_out[[tick, 5]] = contingency_continuity.refinement_probe_samples_evaluated as u64;
            work_out[[tick, 6]] = contingency_continuity.unresolved_interval_count as u64;
            work_out[[tick, 7]] =
                u64::from(contingency_continuity.maximum_subdivision_depth_reached);
            let prediction_end_time =
                tick_time_ns.saturating_add(self.program.timing.control_horizon_ns);
            for (candidate, prediction) in [
                &self.output.primary_root_prediction,
                &self.output.contingency_root_prediction,
            ]
            .into_iter()
            .enumerate()
            {
                let offset = candidate * 24;
                let mut end_pose = bonesaw_core::Transform3::identity();
                let mut end_twist = Motion6::default();
                let mut end_acceleration = SpatialAcceleration6::default();
                prediction
                    .evaluate_into(
                        prediction_end_time,
                        &mut end_pose,
                        &mut end_twist,
                        &mut end_acceleration,
                    )
                    .map_err(value_error)?;
                let mut maximum_abs_twist = Motion6::default();
                prediction
                    .maximum_abs_twist_between_into(
                        tick_time_ns,
                        prediction_end_time,
                        &mut maximum_abs_twist,
                    )
                    .map_err(value_error)?;
                for axis in 0..3 {
                    root_prediction_out[[tick, offset + axis]] = end_pose.translation.vector[axis];
                    root_prediction_out[[tick, offset + 3 + axis]] =
                        end_pose.rotation.scaled_axis()[axis];
                }
                for axis in 0..6 {
                    root_prediction_out[[tick, offset + 6 + axis]] = end_twist.0[axis];
                    root_prediction_out[[tick, offset + 12 + axis]] = maximum_abs_twist.0[axis];
                }
                let error = prediction.error_growth;
                for (column, value) in [
                    error.initial_translation_radius_m,
                    error.translation_velocity_error_bound_mps,
                    error.translation_acceleration_error_bound_mps2,
                    error.initial_rotation_radius_rad,
                    error.angular_velocity_error_bound_radps,
                    error.angular_acceleration_error_bound_radps2,
                ]
                .into_iter()
                .enumerate()
                {
                    root_prediction_out[[tick, offset + 18 + column]] = value;
                }
            }
            timing_out[[tick, 0]] = elapsed_ns;
            timing_out[[tick, 1]] = allocation_calls;
            timing_out[[tick, 2]] = allocated_bytes;
            for coordinate in 0..generalized_dof {
                generalized_acceleration_out[[tick, coordinate]] =
                    self.output.wbc.generalized_acceleration[coordinate];
            }
            for actuator in 0..actuators {
                splice_position_out[[tick, actuator]] = self.output.splice_position[actuator];
                splice_velocity_out[[tick, actuator]] = self.output.splice_velocity[actuator];
                splice_acceleration_out[[tick, actuator]] =
                    self.output.splice_acceleration[actuator];
            }
            if self.output.selection != bonesaw_core::DynamicPlanSelection::Rejected {
                std::mem::swap(&mut self.state_in, &mut self.state_out);
            }
        }
        Ok(())
    }
}

fn solve_status_code(status: SolveStatus) -> u8 {
    match status {
        SolveStatus::Solved => 0,
        SolveStatus::SolvedWithSlack => 1,
        SolveStatus::MaxIterations => 2,
        SolveStatus::PrimalInfeasible => 3,
        SolveStatus::NumericalFailure => 4,
        SolveStatus::InvalidProblem => 5,
    }
}

fn dynamic_status_code(status: bonesaw_core::DynamicStepStatus) -> u8 {
    match status {
        bonesaw_core::DynamicStepStatus::Ok => 0,
        bonesaw_core::DynamicStepStatus::Degraded => 1,
        bonesaw_core::DynamicStepStatus::Contingency => 2,
        bonesaw_core::DynamicStepStatus::Rejected => 3,
    }
}

fn dynamic_selection_code(selection: bonesaw_core::DynamicPlanSelection) -> u8 {
    match selection {
        bonesaw_core::DynamicPlanSelection::Primary => 0,
        bonesaw_core::DynamicPlanSelection::Contingency => 1,
        bonesaw_core::DynamicPlanSelection::Rejected => 2,
    }
}

fn command_tracking_action_code(action: CommandTrackingAction) -> u8 {
    match action {
        CommandTrackingAction::Nominal => 0,
        CommandTrackingAction::Contingency => 1,
        CommandTrackingAction::Rejected => 2,
    }
}

/// Offline morphology witness used by policy-free, simulator-free evals.
#[pyclass(unsendable)]
struct KinematicWitnessSession {
    program: MotionProgram,
    state: RobotState,
    scratch: WholeBodyIkScratch,
    point_targets: Vec<WholeBodyPointIkTarget>,
    jet_targets: Vec<WholeBodyPointJetTarget>,
    target_rotations: Vec<UnitQuaternion<f64>>,
    nominal_posture: DVector<f64>,
    initial_posture: DVector<f64>,
    jet_coordinate_regularization: DVector<f64>,
    joint_acceleration: DVector<f64>,
}

/// Fixed-layout `CpuMirrorF32` boundary used to certify CUDA batch semantics
/// before a device executor is admitted.
#[pyclass(unsendable)]
struct CpuMirrorBatchSession {
    program: MotionProgram,
    executor: CpuMirrorExecutor,
    input: FkBatchInput,
    output: FkBatchOutput,
    jacobian_output: JacobianBatchOutput,
    dynamics_input: DynamicsBatchInput,
    dynamics_output: DynamicsBatchOutput,
    point_output: PointQueryBatchOutput,
    emission_input: EmissionBatchInput,
    emission_output: EmissionBatchOutput,
    exact_solve_input: ExactSolveBatchInput,
    exact_solve_output: ExactSolveBatchOutput,
    exact_solver: CpuExactBatchSolver,
    joint_envelope_input: JointEnvelopeBatchInput,
    joint_envelope_output: JointEnvelopeBatchOutput,
    joint_envelope_executor: CpuJointEnvelopeBatchExecutor,
    exact_dynamic_input: ExactDynamicBatchInput,
    exact_dynamic_output: ExactDynamicBatchOutput,
    exact_dynamic_solver: CpuExactDynamicBatchSolver,
    exact_dynamic_support_patches: Vec<ExactDynamicSupportPatchSpec>,
}

#[pymethods]
impl CpuMirrorBatchSession {
    #[new]
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (
        urdf_path,
        agent_capacity,
        agent_alignment=32,
        point_frame_names=None,
        point_offsets=None,
        point_stable_ids=None,
        point_task_query_slots=None,
        point_task_stable_ids=None,
        point_task_priorities=None,
        point_task_weights=None,
        point_task_bandwidth_hz=None,
        contact_query_slots=None,
        contact_stable_ids=None,
        contact_kinematic_modes=None,
        support_patch_first_contact_slots=None,
        support_patch_contact_counts=None,
        support_patch_stable_ids=None,
        auto_rigid_support_basis=false
    ))]
    fn new(
        urdf_path: &str,
        agent_capacity: usize,
        agent_alignment: usize,
        point_frame_names: Option<Vec<String>>,
        point_offsets: Option<Vec<(f64, f64, f64)>>,
        point_stable_ids: Option<Vec<u32>>,
        point_task_query_slots: Option<Vec<usize>>,
        point_task_stable_ids: Option<Vec<u32>>,
        point_task_priorities: Option<Vec<u8>>,
        point_task_weights: Option<Vec<f64>>,
        point_task_bandwidth_hz: Option<Vec<f64>>,
        contact_query_slots: Option<Vec<usize>>,
        contact_stable_ids: Option<Vec<u32>>,
        contact_kinematic_modes: Option<Vec<u8>>,
        support_patch_first_contact_slots: Option<Vec<usize>>,
        support_patch_contact_counts: Option<Vec<usize>>,
        support_patch_stable_ids: Option<Vec<u32>>,
        auto_rigid_support_basis: bool,
    ) -> PyResult<Self> {
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let point_specs = match (point_frame_names, point_offsets, point_stable_ids) {
            (None, None, None) => Vec::new(),
            (Some(frames), Some(offsets), Some(stable_ids))
                if frames.len() == offsets.len() && frames.len() == stable_ids.len() =>
            {
                let mut specs = Vec::with_capacity(frames.len());
                for ((frame_name, offset), stable_id) in
                    frames.into_iter().zip(offsets).zip(stable_ids)
                {
                    let frame_index = program
                        .model
                        .bodies
                        .iter()
                        .position(|body| body.name == frame_name)
                        .ok_or_else(|| {
                            PyValueError::new_err(format!(
                                "point query refers to unknown body frame {frame_name}"
                            ))
                        })?;
                    specs.push(PointQuerySpec {
                        stable_id,
                        frame_index,
                        point_in_frame: [offset.0, offset.1, offset.2],
                    });
                }
                specs
            }
            _ => {
                return Err(PyValueError::new_err(
                    "point_frame_names, point_offsets, and point_stable_ids must all be omitted or have equal lengths",
                ));
            }
        };
        let point_tasks = match (
            point_task_query_slots,
            point_task_stable_ids,
            point_task_priorities,
            point_task_weights,
            point_task_bandwidth_hz,
        ) {
            (None, None, None, None, None) => Vec::new(),
            (
                Some(query_slots),
                Some(stable_ids),
                Some(priorities),
                Some(weights),
                Some(bandwidths),
            ) if query_slots.len() == stable_ids.len()
                && query_slots.len() == priorities.len()
                && query_slots.len() == weights.len()
                && query_slots.len() == bandwidths.len() =>
            {
                let mut specs = Vec::with_capacity(query_slots.len());
                for ((((query_slot, stable_id), priority), weight), bandwidth_hz) in query_slots
                    .into_iter()
                    .zip(stable_ids)
                    .zip(priorities)
                    .zip(weights)
                    .zip(bandwidths)
                {
                    let point_query_stable_id = point_specs
                        .get(query_slot)
                        .ok_or_else(|| {
                            PyValueError::new_err(format!(
                                "point task {stable_id} refers to point query slot {query_slot}, outside {} slots",
                                point_specs.len()
                            ))
                        })?
                        .stable_id;
                    let priority = match priority {
                        0 => Priority::Invariant,
                        1 => Priority::Viability,
                        2 => Priority::Intent,
                        3 => Priority::Preference,
                        4 => Priority::Style,
                        value => {
                            return Err(PyValueError::new_err(format!(
                                "point task {stable_id} has invalid priority {value}; expected 0..=4"
                            )));
                        }
                    };
                    specs.push(PointAttractorSpec {
                        stable_id,
                        point_query_stable_id,
                        priority,
                        weight,
                        bandwidth_hz,
                    });
                }
                specs
            }
            _ => {
                return Err(PyValueError::new_err(
                    "all point_task_* arguments must be omitted or have equal lengths",
                ));
            }
        };
        let contact_locks = match (contact_query_slots, contact_stable_ids) {
            (None, None) => Vec::new(),
            (Some(query_slots), Some(stable_ids)) if query_slots.len() == stable_ids.len() => {
                let mut specs = Vec::with_capacity(query_slots.len());
                for (query_slot, stable_id) in query_slots.into_iter().zip(stable_ids) {
                    let point_query_stable_id = point_specs
                        .get(query_slot)
                        .ok_or_else(|| {
                            PyValueError::new_err(format!(
                                "contact lock {stable_id} refers to point query slot {query_slot}, outside {} slots",
                                point_specs.len()
                            ))
                        })?
                        .stable_id;
                    specs.push(ContactLockSpec {
                        stable_id,
                        point_query_stable_id,
                    });
                }
                specs
            }
            _ => {
                return Err(PyValueError::new_err(
                    "contact_query_slots and contact_stable_ids must both be omitted or have equal lengths",
                ));
            }
        };
        let support_patch_specs = match (
            support_patch_first_contact_slots,
            support_patch_contact_counts,
            support_patch_stable_ids,
        ) {
            (None, None, None) => Vec::new(),
            (Some(first), Some(counts), Some(ids))
                if first.len() == counts.len() && first.len() == ids.len() =>
            {
                first
                    .into_iter()
                    .zip(counts)
                    .zip(ids)
                    .map(|((first_contact_slot, contact_count), stable_id)| {
                        ExactDynamicSupportPatchSpec {
                            stable_id,
                            first_contact_slot,
                            contact_count,
                        }
                    })
                    .collect()
            }
            _ => {
                return Err(PyValueError::new_err(
                    "all support_patch_* arguments must be omitted or have equal lengths",
                ));
            }
        };
        let contact_kinematic_modes = match (auto_rigid_support_basis, contact_kinematic_modes) {
            (true, Some(_)) => {
                return Err(PyValueError::new_err(
                    "contact_kinematic_modes must be omitted when auto_rigid_support_basis is true",
                ));
            }
            (true, None) => {
                derive_rigid_patch_contact_modes(
                    &point_specs,
                    &contact_locks,
                    &support_patch_specs
                        .iter()
                        .map(|patch| RigidPatchBasisSpec {
                            stable_id: patch.stable_id,
                            first_contact_slot: patch.first_contact_slot,
                            contact_count: patch.contact_count,
                        })
                        .collect::<Vec<_>>(),
                )
                .map_err(value_error)?
                .contact_modes
            }
            (false, None) => vec![ContactKinematicMode::LockedPoint; contact_locks.len()],
            (false, Some(values)) if values.len() == contact_locks.len() => values
                .into_iter()
                .map(|value| match value {
                    0 => Ok(ContactKinematicMode::Disabled),
                    1 => Ok(ContactKinematicMode::LockedPoint),
                    2 => Ok(ContactKinematicMode::NormalPoint),
                    3 => Ok(ContactKinematicMode::RollingPoint),
                    _ => Err(PyValueError::new_err(format!(
                        "invalid contact kinematic mode {value}; expected 0..=3"
                    ))),
                })
                .collect::<PyResult<Vec<_>>>()?,
            (false, Some(values)) => {
                return Err(PyValueError::new_err(format!(
                    "contact_kinematic_modes has {} entries for {} contacts",
                    values.len(),
                    contact_locks.len()
                )));
            }
        };
        let executor = CpuMirrorExecutor::compile_with_contact_modes(
            &program,
            agent_capacity,
            agent_alignment,
            &point_specs,
            &point_tasks,
            &contact_locks,
            &contact_kinematic_modes,
        )
        .map_err(value_error)?;
        let input = FkBatchInput::new(executor.descriptor().layout.clone(), agent_capacity)
            .map_err(value_error)?;
        let output = FkBatchOutput::new(executor.descriptor().layout.clone());
        let jacobian_output = JacobianBatchOutput::new(executor.descriptor().layout.clone());
        let dynamics_input =
            DynamicsBatchInput::new(executor.descriptor().layout.clone(), agent_capacity)
                .map_err(value_error)?;
        let dynamics_output = DynamicsBatchOutput::new(executor.descriptor().layout.clone());
        let point_output = PointQueryBatchOutput::new(executor.descriptor().layout.clone());
        let emission_input =
            EmissionBatchInput::new(executor.descriptor().layout.clone(), agent_capacity)
                .map_err(value_error)?;
        let emission_output = EmissionBatchOutput::new(executor.descriptor().layout.clone());
        let exact_solve_input =
            ExactSolveBatchInput::new(executor.descriptor().layout.clone(), agent_capacity)
                .map_err(value_error)?;
        let exact_solve_output = ExactSolveBatchOutput::new(executor.descriptor().layout.clone());
        let exact_solver = CpuExactBatchSolver::new(executor.descriptor()).map_err(value_error)?;
        let joint_envelope_input =
            JointEnvelopeBatchInput::new(executor.descriptor().layout.clone(), agent_capacity)
                .map_err(value_error)?;
        let joint_envelope_output =
            JointEnvelopeBatchOutput::new(executor.descriptor().layout.clone());
        let joint_envelope_executor =
            CpuJointEnvelopeBatchExecutor::compile(&program, executor.descriptor().layout.clone())
                .map_err(value_error)?;
        let exact_dynamic_input = ExactDynamicBatchInput::new_with_support_patch_count(
            &program,
            executor.descriptor().layout.clone(),
            agent_capacity,
            support_patch_specs.len(),
        )
        .map_err(value_error)?;
        let exact_dynamic_output = ExactDynamicBatchOutput::new(
            executor.descriptor().layout.clone(),
            program.actuation.actuators.len(),
        );
        let exact_dynamic_solver = CpuExactDynamicBatchSolver::new_with_support_patches(
            &program,
            executor.descriptor(),
            &support_patch_specs,
        )
        .map_err(value_error)?;
        Ok(Self {
            program,
            executor,
            input,
            output,
            jacobian_output,
            dynamics_input,
            dynamics_output,
            point_output,
            emission_input,
            emission_output,
            exact_solve_input,
            exact_solve_output,
            exact_solver,
            joint_envelope_input,
            joint_envelope_output,
            joint_envelope_executor,
            exact_dynamic_input,
            exact_dynamic_output,
            exact_dynamic_solver,
            exact_dynamic_support_patches: support_patch_specs,
        })
    }

    #[getter]
    fn agent_capacity(&self) -> usize {
        self.executor.descriptor().layout.agent_capacity
    }

    #[getter]
    fn agent_stride(&self) -> usize {
        self.executor.descriptor().layout.agent_stride
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn generalized_dof(&self) -> usize {
        self.executor
            .descriptor()
            .layout
            .generalized_coordinate_count
    }

    #[getter]
    fn actuator_count(&self) -> usize {
        self.program.actuation.actuators.len()
    }

    #[getter]
    fn jacobian_components(&self) -> usize {
        SPATIAL_COMPONENTS
    }

    #[getter]
    fn point_query_count(&self) -> usize {
        self.executor.descriptor().layout.point_query_count
    }

    #[getter]
    fn point_stable_ids(&self) -> Vec<u32> {
        self.executor
            .descriptor()
            .point_queries
            .iter()
            .map(|query| query.stable_id)
            .collect()
    }

    #[getter]
    fn point_frame_names(&self) -> Vec<&str> {
        self.executor
            .descriptor()
            .point_queries
            .iter()
            .map(|query| self.program.model.bodies[query.frame_index].name.as_str())
            .collect()
    }

    #[getter]
    fn point_offsets(&self) -> Vec<(f32, f32, f32)> {
        self.executor
            .descriptor()
            .point_queries
            .iter()
            .map(|query| {
                let [x, y, z] = query.point_in_frame();
                (x, y, z)
            })
            .collect()
    }

    #[getter]
    fn point_task_count(&self) -> usize {
        self.executor.descriptor().layout.point_task_count
    }

    #[getter]
    fn point_task_stable_ids(&self) -> Vec<u32> {
        self.executor
            .descriptor()
            .point_tasks
            .iter()
            .map(|task| task.stable_id)
            .collect()
    }

    #[getter]
    fn point_task_query_slots(&self) -> Vec<usize> {
        self.executor
            .descriptor()
            .point_tasks
            .iter()
            .map(|task| task.point_query_slot)
            .collect()
    }

    #[getter]
    fn point_task_priorities(&self) -> Vec<u8> {
        self.executor
            .descriptor()
            .point_tasks
            .iter()
            .map(|task| task.priority as u8)
            .collect()
    }

    #[getter]
    fn point_task_weights(&self) -> Vec<f32> {
        self.executor
            .descriptor()
            .point_tasks
            .iter()
            .copied()
            .map(|task| task.weight())
            .collect()
    }

    #[getter]
    fn point_task_bandwidth_hz(&self) -> Vec<f32> {
        self.executor
            .descriptor()
            .point_tasks
            .iter()
            .copied()
            .map(|task| task.bandwidth_hz())
            .collect()
    }

    #[getter]
    fn contact_lock_count(&self) -> usize {
        self.executor.descriptor().layout.contact_lock_count
    }

    #[getter]
    fn contact_lock_stable_ids(&self) -> Vec<u32> {
        self.executor
            .descriptor()
            .contact_locks
            .iter()
            .map(|contact| contact.stable_id)
            .collect()
    }

    #[getter]
    fn contact_lock_query_slots(&self) -> Vec<usize> {
        self.executor
            .descriptor()
            .contact_locks
            .iter()
            .map(|contact| contact.point_query_slot)
            .collect()
    }

    #[getter]
    fn contact_kinematic_modes(&self) -> Vec<u8> {
        self.executor
            .descriptor()
            .contact_locks
            .iter()
            .map(|contact| contact.kinematic_mode as u8)
            .collect()
    }

    #[getter]
    fn support_patch_count(&self) -> usize {
        self.exact_dynamic_support_patches.len()
    }

    #[getter]
    fn support_patch_stable_ids(&self) -> Vec<u32> {
        self.exact_dynamic_support_patches
            .iter()
            .map(|patch| patch.stable_id)
            .collect()
    }

    #[getter]
    fn body_names(&self) -> Vec<&str> {
        self.program
            .model
            .bodies
            .iter()
            .map(|body| body.name.as_str())
            .collect()
    }

    #[getter]
    fn joint_names(&self) -> Vec<&str> {
        self.program
            .model
            .joints
            .iter()
            .filter(|joint| joint.coordinate.is_some())
            .map(|joint| joint.name.as_str())
            .collect()
    }

    #[getter]
    fn backend_fingerprint_json(&self) -> PyResult<String> {
        serde_json::to_string(self.executor.fingerprint()).map_err(value_error)
    }

    #[getter]
    fn kernel_manifest_bits(&self) -> u64 {
        self.executor.descriptor().manifest.bits()
    }

    #[getter]
    fn kernel_abi_version(&self) -> u32 {
        self.executor.descriptor().kernel_abi_version
    }

    #[getter]
    fn cuda_device_executor_available(&self) -> bool {
        false
    }

    #[getter]
    fn exact_solve_semantics(&self) -> &'static str {
        "StrictLexicographic"
    }

    #[getter]
    fn exact_solve_scalar_format(&self) -> &'static str {
        "f64"
    }

    #[getter]
    fn priority_level_count(&self) -> usize {
        PRIORITY_LEVEL_COUNT
    }

    #[getter]
    fn joint_envelope_semantics(&self) -> &'static str {
        "NextTickStoppingDistance"
    }

    /// Derive fixed-shape generalized-acceleration bounds from authored joint
    /// position/velocity limits and explicit per-agent acceleration authority.
    #[allow(clippy::too_many_arguments)]
    fn run_joint_envelope_batch(
        &mut self,
        py: Python<'_>,
        joint_position: PyReadonlyArray2<'_, f64>,
        joint_velocity: PyReadonlyArray2<'_, f64>,
        enabled: PyReadonlyArray1<'_, u8>,
        dt_seconds: PyReadonlyArray1<'_, f64>,
        maximum_joint_acceleration: PyReadonlyArray2<'_, f64>,
        mut acceleration_lower_out: PyReadwriteArray2<'_, f64>,
        mut acceleration_upper_out: PyReadwriteArray2<'_, f64>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut minimum_position_headroom_out: PyReadwriteArray1<'_, f64>,
        mut minimum_velocity_headroom_out: PyReadwriteArray1<'_, f64>,
        mut limiting_position_coordinate_out: PyReadwriteArray1<'_, u32>,
        mut limiting_velocity_coordinate_out: PyReadwriteArray1<'_, u32>,
        mut recovery_coordinate_count_out: PyReadwriteArray1<'_, u32>,
        mut execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let joint_position = joint_position.as_array();
        let joint_velocity = joint_velocity.as_array();
        let enabled = enabled.as_slice()?;
        let dt_seconds = dt_seconds.as_slice()?;
        let maximum_joint_acceleration = maximum_joint_acceleration.as_array();
        let mut acceleration_lower_out = acceleration_lower_out.as_array_mut();
        let mut acceleration_upper_out = acceleration_upper_out.as_array_mut();
        let status_out = status_out.as_slice_mut()?;
        let minimum_position_headroom_out = minimum_position_headroom_out.as_slice_mut()?;
        let minimum_velocity_headroom_out = minimum_velocity_headroom_out.as_slice_mut()?;
        let limiting_position_coordinate_out = limiting_position_coordinate_out.as_slice_mut()?;
        let limiting_velocity_coordinate_out = limiting_velocity_coordinate_out.as_slice_mut()?;
        let recovery_coordinate_count_out = recovery_coordinate_count_out.as_slice_mut()?;
        let execute_ns_out = execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let layout = &self.executor.descriptor().layout;
        let agents = layout.agent_capacity;
        let dof = layout.coordinate_count;
        let generalized_dof = layout.generalized_coordinate_count;
        if joint_position.shape() != [agents, dof]
            || joint_velocity.shape() != [agents, dof]
            || enabled.len() != agents
            || dt_seconds.len() != agents
            || maximum_joint_acceleration.shape() != [agents, dof]
            || acceleration_lower_out.shape() != [agents, generalized_dof]
            || acceleration_upper_out.shape() != [agents, generalized_dof]
            || status_out.len() != agents
            || minimum_position_headroom_out.len() != agents
            || minimum_velocity_headroom_out.len() != agents
            || limiting_position_coordinate_out.len() != agents
            || limiting_velocity_coordinate_out.len() != agents
            || recovery_coordinate_count_out.len() != agents
            || execute_ns_out.len() != 1
            || allocation_calls_out.len() != 1
            || allocated_bytes_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "joint envelope input/output array shape mismatch",
            ));
        }
        for agent in 0..agents {
            self.joint_envelope_input.enabled_mut()[agent] = enabled[agent];
            self.joint_envelope_input.dt_seconds_mut()[agent] = dt_seconds[agent];
            for coordinate in 0..dof {
                let index = layout.q_index(coordinate, agent);
                self.joint_envelope_input.position_soa_mut()[index] =
                    joint_position[[agent, coordinate]];
                self.joint_envelope_input.velocity_soa_mut()[index] =
                    joint_velocity[[agent, coordinate]];
                self.joint_envelope_input.maximum_acceleration_soa_mut()[index] =
                    maximum_joint_acceleration[[agent, coordinate]];
            }
        }
        let before = allocation_snapshot();
        let started = Instant::now();
        py.detach(|| {
            self.joint_envelope_executor
                .execute_into(&self.joint_envelope_input, &mut self.joint_envelope_output)
                .map_err(value_error)
        })?;
        execute_ns_out[0] = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            status_out[agent] = self.joint_envelope_output.status()[agent] as u8;
            minimum_position_headroom_out[agent] =
                self.joint_envelope_output.minimum_position_headroom()[agent];
            minimum_velocity_headroom_out[agent] =
                self.joint_envelope_output.minimum_velocity_headroom()[agent];
            limiting_position_coordinate_out[agent] =
                self.joint_envelope_output.limiting_position_coordinate()[agent];
            limiting_velocity_coordinate_out[agent] =
                self.joint_envelope_output.limiting_velocity_coordinate()[agent];
            recovery_coordinate_count_out[agent] =
                self.joint_envelope_output.recovery_coordinate_count()[agent];
            for coordinate in 0..generalized_dof {
                let index = layout.generalized_index(coordinate, agent);
                acceleration_lower_out[[agent, coordinate]] =
                    self.joint_envelope_output.lower_soa()[index];
                acceleration_upper_out[[agent, coordinate]] =
                    self.joint_envelope_output.upper_soa()[index];
            }
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn run_fk_batch(
        &mut self,
        py: Python<'_>,
        q: PyReadonlyArray2<'_, f32>,
        root_pose: PyReadonlyArray2<'_, f32>,
        mut body_pose_out: PyReadwriteArray3<'_, f32>,
        mut center_of_mass_out: PyReadwriteArray2<'_, f32>,
        mut total_mass_out: PyReadwriteArray1<'_, f32>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let q = q.as_array();
        let root_pose = root_pose.as_array();
        let mut body_pose_out = body_pose_out.as_array_mut();
        let mut center_of_mass_out = center_of_mass_out.as_array_mut();
        let total_mass_out = total_mass_out.as_slice_mut()?;
        let status_out = status_out.as_slice_mut()?;
        let execute_ns_out = execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let layout = &self.executor.descriptor().layout;
        let agents = layout.agent_capacity;
        let dof = layout.coordinate_count;
        let bodies = layout.body_count;
        let stride = layout.agent_stride;
        if q.shape() != [agents, dof]
            || root_pose.shape() != [agents, POSE_COMPONENTS]
            || body_pose_out.shape() != [agents, bodies, POSE_COMPONENTS]
            || center_of_mass_out.shape() != [agents, 3]
            || total_mass_out.len() != agents
            || status_out.len() != agents
            || execute_ns_out.len() != 1
            || allocation_calls_out.len() != 1
            || allocated_bytes_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "CPU mirror input/output array shape mismatch",
            ));
        }
        for agent in 0..agents {
            for coordinate in 0..dof {
                self.input.q_soa_mut()[coordinate * stride + agent] = q[[agent, coordinate]];
            }
            for component in 0..POSE_COMPONENTS {
                self.input.root_pose_soa_mut()[component * stride + agent] =
                    root_pose[[agent, component]];
            }
        }
        let before = allocation_snapshot();
        let started = Instant::now();
        py.detach(|| {
            self.executor
                .execute_into(&self.input, &mut self.output)
                .map_err(value_error)
        })?;
        execute_ns_out[0] = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            for body in 0..bodies {
                for component in 0..POSE_COMPONENTS {
                    body_pose_out[[agent, body, component]] = self.output.body_pose_soa()
                        [(body * POSE_COMPONENTS + component) * stride + agent];
                }
            }
            for component in 0..3 {
                center_of_mass_out[[agent, component]] =
                    self.output.center_of_mass_soa()[component * stride + agent];
            }
            total_mass_out[agent] = self.output.total_mass()[agent];
            status_out[agent] = self.output.agent_status()[agent] as u8;
        }
        Ok(())
    }

    /// Run the fixed-layout FK/CoM stage followed by floating frame-origin and
    /// CoM Jacobians. Timers surround the two Rust stages independently;
    /// Python/NumPy marshalling remains outside both measurements.
    #[allow(clippy::too_many_arguments)]
    fn run_kinematics_batch(
        &mut self,
        py: Python<'_>,
        q: PyReadonlyArray2<'_, f32>,
        root_pose: PyReadonlyArray2<'_, f32>,
        mut body_pose_out: PyReadwriteArray3<'_, f32>,
        mut center_of_mass_out: PyReadwriteArray2<'_, f32>,
        mut total_mass_out: PyReadwriteArray1<'_, f32>,
        mut frame_jacobian_out: PyReadwriteArray4<'_, f32>,
        mut center_of_mass_jacobian_out: PyReadwriteArray3<'_, f32>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut fk_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut jacobian_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let q = q.as_array();
        let root_pose = root_pose.as_array();
        let mut body_pose_out = body_pose_out.as_array_mut();
        let mut center_of_mass_out = center_of_mass_out.as_array_mut();
        let total_mass_out = total_mass_out.as_slice_mut()?;
        let mut frame_jacobian_out = frame_jacobian_out.as_array_mut();
        let mut center_of_mass_jacobian_out = center_of_mass_jacobian_out.as_array_mut();
        let status_out = status_out.as_slice_mut()?;
        let fk_execute_ns_out = fk_execute_ns_out.as_slice_mut()?;
        let jacobian_execute_ns_out = jacobian_execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let agents = self.executor.descriptor().layout.agent_capacity;
        let dof = self.executor.descriptor().layout.coordinate_count;
        let generalized_dof = self
            .executor
            .descriptor()
            .layout
            .generalized_coordinate_count;
        let bodies = self.executor.descriptor().layout.body_count;
        let stride = self.executor.descriptor().layout.agent_stride;
        if q.shape() != [agents, dof]
            || root_pose.shape() != [agents, POSE_COMPONENTS]
            || body_pose_out.shape() != [agents, bodies, POSE_COMPONENTS]
            || center_of_mass_out.shape() != [agents, 3]
            || total_mass_out.len() != agents
            || frame_jacobian_out.shape() != [agents, bodies, SPATIAL_COMPONENTS, generalized_dof]
            || center_of_mass_jacobian_out.shape() != [agents, 3, generalized_dof]
            || status_out.len() != agents
            || fk_execute_ns_out.len() != 1
            || jacobian_execute_ns_out.len() != 1
            || allocation_calls_out.len() != 1
            || allocated_bytes_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "CPU mirror kinematics input/output array shape mismatch",
            ));
        }
        for agent in 0..agents {
            for coordinate in 0..dof {
                self.input.q_soa_mut()[coordinate * stride + agent] = q[[agent, coordinate]];
            }
            for component in 0..POSE_COMPONENTS {
                self.input.root_pose_soa_mut()[component * stride + agent] =
                    root_pose[[agent, component]];
            }
        }
        let before = allocation_snapshot();
        let (fk_execute_ns, jacobian_execute_ns) = py.detach(|| {
            let fk_started = Instant::now();
            self.executor
                .execute_into(&self.input, &mut self.output)
                .map_err(value_error)?;
            let fk_execute_ns = fk_started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let jacobian_started = Instant::now();
            self.executor
                .execute_jacobians_into(&self.output, &mut self.jacobian_output)
                .map_err(value_error)?;
            let jacobian_execute_ns =
                jacobian_started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            Ok::<_, PyErr>((fk_execute_ns, jacobian_execute_ns))
        })?;
        fk_execute_ns_out[0] = fk_execute_ns;
        jacobian_execute_ns_out[0] = jacobian_execute_ns;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            for body in 0..bodies {
                for component in 0..POSE_COMPONENTS {
                    body_pose_out[[agent, body, component]] = self.output.body_pose_soa()
                        [(body * POSE_COMPONENTS + component) * stride + agent];
                }
                for component in 0..SPATIAL_COMPONENTS {
                    for coordinate in 0..generalized_dof {
                        frame_jacobian_out[[agent, body, component, coordinate]] = self
                            .jacobian_output
                            .frame_jacobian_soa()[((body * SPATIAL_COMPONENTS + component)
                            * generalized_dof
                            + coordinate)
                            * stride
                            + agent];
                    }
                }
            }
            for component in 0..3 {
                center_of_mass_out[[agent, component]] =
                    self.output.center_of_mass_soa()[component * stride + agent];
                for coordinate in 0..generalized_dof {
                    center_of_mass_jacobian_out[[agent, component, coordinate]] =
                        self.jacobian_output.center_of_mass_jacobian_soa()
                            [(component * generalized_dof + coordinate) * stride + agent];
                }
            }
            total_mass_out[agent] = self.output.total_mass()[agent];
            status_out[agent] = self.jacobian_output.agent_status()[agent] as u8;
        }
        Ok(())
    }

    /// Run FK, Jacobians, then floating mass matrix, bias force, and centroidal
    /// map. The generalized tangent is `[root angular xyz; root linear xyz;
    /// joints]`, all root components world-expressed. Each timer surrounds only
    /// its Rust stage; NumPy marshalling is excluded.
    #[allow(clippy::too_many_arguments)]
    fn run_model_products_batch(
        &mut self,
        py: Python<'_>,
        q: PyReadonlyArray2<'_, f32>,
        root_pose: PyReadonlyArray2<'_, f32>,
        generalized_velocity: PyReadonlyArray2<'_, f32>,
        gravity_world: PyReadonlyArray2<'_, f32>,
        mut mass_matrix_out: PyReadwriteArray3<'_, f32>,
        mut bias_force_out: PyReadwriteArray2<'_, f32>,
        mut centroidal_map_out: PyReadwriteArray3<'_, f32>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut fk_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut jacobian_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut dynamics_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let q = q.as_array();
        let root_pose = root_pose.as_array();
        let generalized_velocity = generalized_velocity.as_array();
        let gravity_world = gravity_world.as_array();
        let mut mass_matrix_out = mass_matrix_out.as_array_mut();
        let mut bias_force_out = bias_force_out.as_array_mut();
        let mut centroidal_map_out = centroidal_map_out.as_array_mut();
        let status_out = status_out.as_slice_mut()?;
        let fk_execute_ns_out = fk_execute_ns_out.as_slice_mut()?;
        let jacobian_execute_ns_out = jacobian_execute_ns_out.as_slice_mut()?;
        let dynamics_execute_ns_out = dynamics_execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let layout = &self.executor.descriptor().layout;
        let agents = layout.agent_capacity;
        let dof = layout.coordinate_count;
        let generalized_dof = layout.generalized_coordinate_count;
        let stride = layout.agent_stride;
        if q.shape() != [agents, dof]
            || root_pose.shape() != [agents, POSE_COMPONENTS]
            || generalized_velocity.shape() != [agents, generalized_dof]
            || gravity_world.shape() != [agents, 3]
            || mass_matrix_out.shape() != [agents, generalized_dof, generalized_dof]
            || bias_force_out.shape() != [agents, generalized_dof]
            || centroidal_map_out.shape() != [agents, SPATIAL_COMPONENTS, generalized_dof]
            || status_out.len() != agents
            || fk_execute_ns_out.len() != 1
            || jacobian_execute_ns_out.len() != 1
            || dynamics_execute_ns_out.len() != 1
            || allocation_calls_out.len() != 1
            || allocated_bytes_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "CPU mirror model-products input/output array shape mismatch",
            ));
        }
        for agent in 0..agents {
            for coordinate in 0..dof {
                self.input.q_soa_mut()[coordinate * stride + agent] = q[[agent, coordinate]];
            }
            for component in 0..POSE_COMPONENTS {
                self.input.root_pose_soa_mut()[component * stride + agent] =
                    root_pose[[agent, component]];
            }
            for coordinate in 0..generalized_dof {
                self.dynamics_input.generalized_velocity_soa_mut()[coordinate * stride + agent] =
                    generalized_velocity[[agent, coordinate]];
            }
            for component in 0..3 {
                self.dynamics_input.gravity_world_soa_mut()[component * stride + agent] =
                    gravity_world[[agent, component]];
            }
        }
        let before = allocation_snapshot();
        let (fk_execute_ns, jacobian_execute_ns, dynamics_execute_ns) = py.detach(|| {
            let started = Instant::now();
            self.executor
                .execute_into(&self.input, &mut self.output)
                .map_err(value_error)?;
            let fk_execute_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_jacobians_into(&self.output, &mut self.jacobian_output)
                .map_err(value_error)?;
            let jacobian_execute_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_dynamics_into(
                    &self.dynamics_input,
                    &self.output,
                    &self.jacobian_output,
                    &mut self.dynamics_output,
                )
                .map_err(value_error)?;
            let dynamics_execute_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            Ok::<_, PyErr>((fk_execute_ns, jacobian_execute_ns, dynamics_execute_ns))
        })?;
        fk_execute_ns_out[0] = fk_execute_ns;
        jacobian_execute_ns_out[0] = jacobian_execute_ns;
        dynamics_execute_ns_out[0] = dynamics_execute_ns;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            for row in 0..generalized_dof {
                bias_force_out[[agent, row]] =
                    self.dynamics_output.bias_force_soa()[row * stride + agent];
                for column in 0..generalized_dof {
                    mass_matrix_out[[agent, row, column]] = self.dynamics_output.mass_matrix_soa()
                        [(row * generalized_dof + column) * stride + agent];
                }
            }
            for component in 0..SPATIAL_COMPONENTS {
                for coordinate in 0..generalized_dof {
                    centroidal_map_out[[agent, component, coordinate]] =
                        self.dynamics_output.centroidal_map_soa()
                            [(component * generalized_dof + coordinate) * stride + agent];
                }
            }
            status_out[agent] = self.dynamics_output.agent_status()[agent] as u8;
        }
        Ok(())
    }

    /// Execute the complete admitted model-product pipeline followed by every
    /// compiler-resolved point slot. Python/NumPy marshalling is excluded from
    /// all four stage timers.
    #[allow(clippy::too_many_arguments)]
    fn run_point_queries_batch(
        &mut self,
        py: Python<'_>,
        q: PyReadonlyArray2<'_, f32>,
        root_pose: PyReadonlyArray2<'_, f32>,
        generalized_velocity: PyReadonlyArray2<'_, f32>,
        gravity_world: PyReadonlyArray2<'_, f32>,
        mut point_position_out: PyReadwriteArray3<'_, f32>,
        mut point_jacobian_out: PyReadwriteArray4<'_, f32>,
        mut point_bias_acceleration_out: PyReadwriteArray3<'_, f32>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut fk_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut jacobian_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut dynamics_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut point_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let q = q.as_array();
        let root_pose = root_pose.as_array();
        let generalized_velocity = generalized_velocity.as_array();
        let gravity_world = gravity_world.as_array();
        let mut point_position_out = point_position_out.as_array_mut();
        let mut point_jacobian_out = point_jacobian_out.as_array_mut();
        let mut point_bias_acceleration_out = point_bias_acceleration_out.as_array_mut();
        let status_out = status_out.as_slice_mut()?;
        let fk_execute_ns_out = fk_execute_ns_out.as_slice_mut()?;
        let jacobian_execute_ns_out = jacobian_execute_ns_out.as_slice_mut()?;
        let dynamics_execute_ns_out = dynamics_execute_ns_out.as_slice_mut()?;
        let point_execute_ns_out = point_execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let layout = &self.executor.descriptor().layout;
        let agents = layout.agent_capacity;
        let dof = layout.coordinate_count;
        let generalized_dof = layout.generalized_coordinate_count;
        let slots = layout.point_query_count;
        let stride = layout.agent_stride;
        if q.shape() != [agents, dof]
            || root_pose.shape() != [agents, POSE_COMPONENTS]
            || generalized_velocity.shape() != [agents, generalized_dof]
            || gravity_world.shape() != [agents, 3]
            || point_position_out.shape() != [agents, slots, 3]
            || point_jacobian_out.shape() != [agents, slots, 3, generalized_dof]
            || point_bias_acceleration_out.shape() != [agents, slots, 3]
            || status_out.len() != agents
            || fk_execute_ns_out.len() != 1
            || jacobian_execute_ns_out.len() != 1
            || dynamics_execute_ns_out.len() != 1
            || point_execute_ns_out.len() != 1
            || allocation_calls_out.len() != 1
            || allocated_bytes_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "CPU mirror point-query input/output array shape mismatch",
            ));
        }
        for agent in 0..agents {
            for coordinate in 0..dof {
                self.input.q_soa_mut()[coordinate * stride + agent] = q[[agent, coordinate]];
            }
            for component in 0..POSE_COMPONENTS {
                self.input.root_pose_soa_mut()[component * stride + agent] =
                    root_pose[[agent, component]];
            }
            for coordinate in 0..generalized_dof {
                self.dynamics_input.generalized_velocity_soa_mut()[coordinate * stride + agent] =
                    generalized_velocity[[agent, coordinate]];
            }
            for component in 0..3 {
                self.dynamics_input.gravity_world_soa_mut()[component * stride + agent] =
                    gravity_world[[agent, component]];
            }
        }
        let before = allocation_snapshot();
        let (fk_ns, jacobian_ns, dynamics_ns, point_ns) = py.detach(|| {
            let started = Instant::now();
            self.executor
                .execute_into(&self.input, &mut self.output)
                .map_err(value_error)?;
            let fk_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_jacobians_into(&self.output, &mut self.jacobian_output)
                .map_err(value_error)?;
            let jacobian_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_dynamics_into(
                    &self.dynamics_input,
                    &self.output,
                    &self.jacobian_output,
                    &mut self.dynamics_output,
                )
                .map_err(value_error)?;
            let dynamics_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_point_queries_into(
                    &self.output,
                    &self.jacobian_output,
                    &self.dynamics_output,
                    &mut self.point_output,
                )
                .map_err(value_error)?;
            let point_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            Ok::<_, PyErr>((fk_ns, jacobian_ns, dynamics_ns, point_ns))
        })?;
        fk_execute_ns_out[0] = fk_ns;
        jacobian_execute_ns_out[0] = jacobian_ns;
        dynamics_execute_ns_out[0] = dynamics_ns;
        point_execute_ns_out[0] = point_ns;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            for slot in 0..slots {
                for component in 0..3 {
                    point_position_out[[agent, slot, component]] = self
                        .point_output
                        .point_position_soa()[layout.point_position_index(slot, component, agent)];
                    point_bias_acceleration_out[[agent, slot, component]] =
                        self.point_output.point_bias_acceleration_soa()
                            [layout.point_position_index(slot, component, agent)];
                    for coordinate in 0..generalized_dof {
                        point_jacobian_out[[agent, slot, component, coordinate]] =
                            self.point_output.point_jacobian_soa()
                                [layout.point_jacobian_index(slot, component, coordinate, agent)];
                    }
                }
            }
            status_out[agent] = self.point_output.agent_status()[agent] as u8;
        }
        Ok(())
    }

    /// Execute the admitted model-product pipeline and lower fixed-shape point
    /// attractors plus contact locks into acceleration rows. Python/NumPy
    /// marshalling is excluded from all five stage timers.
    #[allow(clippy::too_many_arguments)]
    fn run_emission_batch(
        &mut self,
        py: Python<'_>,
        q: PyReadonlyArray2<'_, f32>,
        root_pose: PyReadonlyArray2<'_, f32>,
        generalized_velocity: PyReadonlyArray2<'_, f32>,
        gravity_world: PyReadonlyArray2<'_, f32>,
        point_task_active: PyReadonlyArray2<'_, u8>,
        point_target_position: PyReadonlyArray3<'_, f32>,
        point_target_velocity: PyReadonlyArray3<'_, f32>,
        point_target_acceleration: PyReadonlyArray3<'_, f32>,
        contact_lock_active: PyReadonlyArray2<'_, u8>,
        contact_desired_acceleration: PyReadonlyArray3<'_, f32>,
        mut point_task_active_out: PyReadwriteArray2<'_, u8>,
        mut point_task_position_error_out: PyReadwriteArray3<'_, f32>,
        mut point_task_velocity_error_out: PyReadwriteArray3<'_, f32>,
        mut point_task_desired_acceleration_out: PyReadwriteArray3<'_, f32>,
        mut point_task_jacobian_out: PyReadwriteArray4<'_, f32>,
        mut point_task_rhs_out: PyReadwriteArray3<'_, f32>,
        mut contact_lock_active_out: PyReadwriteArray2<'_, u8>,
        mut contact_lock_jacobian_out: PyReadwriteArray4<'_, f32>,
        mut contact_lock_rhs_out: PyReadwriteArray3<'_, f32>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut fk_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut jacobian_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut dynamics_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut point_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut emission_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let q = q.as_array();
        let root_pose = root_pose.as_array();
        let generalized_velocity = generalized_velocity.as_array();
        let gravity_world = gravity_world.as_array();
        let point_task_active = point_task_active.as_array();
        let point_target_position = point_target_position.as_array();
        let point_target_velocity = point_target_velocity.as_array();
        let point_target_acceleration = point_target_acceleration.as_array();
        let contact_lock_active = contact_lock_active.as_array();
        let contact_desired_acceleration = contact_desired_acceleration.as_array();
        let mut point_task_active_out = point_task_active_out.as_array_mut();
        let mut point_task_position_error_out = point_task_position_error_out.as_array_mut();
        let mut point_task_velocity_error_out = point_task_velocity_error_out.as_array_mut();
        let mut point_task_desired_acceleration_out =
            point_task_desired_acceleration_out.as_array_mut();
        let mut point_task_jacobian_out = point_task_jacobian_out.as_array_mut();
        let mut point_task_rhs_out = point_task_rhs_out.as_array_mut();
        let mut contact_lock_active_out = contact_lock_active_out.as_array_mut();
        let mut contact_lock_jacobian_out = contact_lock_jacobian_out.as_array_mut();
        let mut contact_lock_rhs_out = contact_lock_rhs_out.as_array_mut();
        let status_out = status_out.as_slice_mut()?;
        let fk_execute_ns_out = fk_execute_ns_out.as_slice_mut()?;
        let jacobian_execute_ns_out = jacobian_execute_ns_out.as_slice_mut()?;
        let dynamics_execute_ns_out = dynamics_execute_ns_out.as_slice_mut()?;
        let point_execute_ns_out = point_execute_ns_out.as_slice_mut()?;
        let emission_execute_ns_out = emission_execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let layout = &self.executor.descriptor().layout;
        let agents = layout.agent_capacity;
        let dof = layout.coordinate_count;
        let generalized_dof = layout.generalized_coordinate_count;
        let tasks = layout.point_task_count;
        let contacts = layout.contact_lock_count;
        let stride = layout.agent_stride;
        if q.shape() != [agents, dof]
            || root_pose.shape() != [agents, POSE_COMPONENTS]
            || generalized_velocity.shape() != [agents, generalized_dof]
            || gravity_world.shape() != [agents, 3]
            || point_task_active.shape() != [agents, tasks]
            || point_target_position.shape() != [agents, tasks, 3]
            || point_target_velocity.shape() != [agents, tasks, 3]
            || point_target_acceleration.shape() != [agents, tasks, 3]
            || contact_lock_active.shape() != [agents, contacts]
            || contact_desired_acceleration.shape() != [agents, contacts, 3]
            || point_task_active_out.shape() != [agents, tasks]
            || point_task_position_error_out.shape() != [agents, tasks, 3]
            || point_task_velocity_error_out.shape() != [agents, tasks, 3]
            || point_task_desired_acceleration_out.shape() != [agents, tasks, 3]
            || point_task_jacobian_out.shape() != [agents, tasks, 3, generalized_dof]
            || point_task_rhs_out.shape() != [agents, tasks, 3]
            || contact_lock_active_out.shape() != [agents, contacts]
            || contact_lock_jacobian_out.shape() != [agents, contacts, 3, generalized_dof]
            || contact_lock_rhs_out.shape() != [agents, contacts, 3]
            || status_out.len() != agents
            || fk_execute_ns_out.len() != 1
            || jacobian_execute_ns_out.len() != 1
            || dynamics_execute_ns_out.len() != 1
            || point_execute_ns_out.len() != 1
            || emission_execute_ns_out.len() != 1
            || allocation_calls_out.len() != 1
            || allocated_bytes_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "CPU mirror emission input/output array shape mismatch",
            ));
        }
        for agent in 0..agents {
            for coordinate in 0..dof {
                self.input.q_soa_mut()[layout.q_index(coordinate, agent)] = q[[agent, coordinate]];
            }
            for component in 0..POSE_COMPONENTS {
                self.input.root_pose_soa_mut()[layout.root_pose_index(component, agent)] =
                    root_pose[[agent, component]];
            }
            for coordinate in 0..generalized_dof {
                self.dynamics_input.generalized_velocity_soa_mut()
                    [layout.generalized_index(coordinate, agent)] =
                    generalized_velocity[[agent, coordinate]];
            }
            for component in 0..3 {
                self.dynamics_input.gravity_world_soa_mut()[component * stride + agent] =
                    gravity_world[[agent, component]];
            }
            for slot in 0..tasks {
                self.emission_input.point_task_active_soa_mut()
                    [layout.task_active_index(slot, agent)] = point_task_active[[agent, slot]];
                for component in 0..3 {
                    let index = layout.task_vector_index(slot, component, agent);
                    self.emission_input.point_target_position_soa_mut()[index] =
                        point_target_position[[agent, slot, component]];
                    self.emission_input.point_target_velocity_soa_mut()[index] =
                        point_target_velocity[[agent, slot, component]];
                    self.emission_input.point_target_acceleration_soa_mut()[index] =
                        point_target_acceleration[[agent, slot, component]];
                }
            }
            for slot in 0..contacts {
                self.emission_input.contact_lock_active_soa_mut()
                    [layout.contact_active_index(slot, agent)] = contact_lock_active[[agent, slot]];
                for component in 0..3 {
                    self.emission_input.contact_desired_acceleration_soa_mut()
                        [layout.contact_vector_index(slot, component, agent)] =
                        contact_desired_acceleration[[agent, slot, component]];
                }
            }
        }
        let before = allocation_snapshot();
        let (fk_ns, jacobian_ns, dynamics_ns, point_ns, emission_ns) = py.detach(|| {
            let started = Instant::now();
            self.executor
                .execute_into(&self.input, &mut self.output)
                .map_err(value_error)?;
            let fk_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_jacobians_into(&self.output, &mut self.jacobian_output)
                .map_err(value_error)?;
            let jacobian_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_dynamics_into(
                    &self.dynamics_input,
                    &self.output,
                    &self.jacobian_output,
                    &mut self.dynamics_output,
                )
                .map_err(value_error)?;
            let dynamics_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_point_queries_into(
                    &self.output,
                    &self.jacobian_output,
                    &self.dynamics_output,
                    &mut self.point_output,
                )
                .map_err(value_error)?;
            let point_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let started = Instant::now();
            self.executor
                .execute_emission_into(
                    &self.emission_input,
                    &self.dynamics_input,
                    &self.point_output,
                    &mut self.emission_output,
                )
                .map_err(value_error)?;
            let emission_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            Ok::<_, PyErr>((fk_ns, jacobian_ns, dynamics_ns, point_ns, emission_ns))
        })?;
        fk_execute_ns_out[0] = fk_ns;
        jacobian_execute_ns_out[0] = jacobian_ns;
        dynamics_execute_ns_out[0] = dynamics_ns;
        point_execute_ns_out[0] = point_ns;
        emission_execute_ns_out[0] = emission_ns;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            for slot in 0..tasks {
                point_task_active_out[[agent, slot]] = self.emission_output.point_task_active_soa()
                    [layout.task_active_index(slot, agent)];
                for component in 0..3 {
                    let index = layout.task_vector_index(slot, component, agent);
                    point_task_position_error_out[[agent, slot, component]] =
                        self.emission_output.point_task_position_error_soa()[index];
                    point_task_velocity_error_out[[agent, slot, component]] =
                        self.emission_output.point_task_velocity_error_soa()[index];
                    point_task_desired_acceleration_out[[agent, slot, component]] =
                        self.emission_output.point_task_desired_acceleration_soa()[index];
                    point_task_rhs_out[[agent, slot, component]] =
                        self.emission_output.point_task_rhs_soa()[index];
                    for coordinate in 0..generalized_dof {
                        point_task_jacobian_out[[agent, slot, component, coordinate]] =
                            self.emission_output.point_task_jacobian_soa()
                                [layout.task_jacobian_index(slot, component, coordinate, agent)];
                    }
                }
            }
            for slot in 0..contacts {
                contact_lock_active_out[[agent, slot]] = self
                    .emission_output
                    .contact_lock_active_soa()[layout.contact_active_index(slot, agent)];
                for component in 0..3 {
                    contact_lock_rhs_out[[agent, slot, component]] =
                        self.emission_output.contact_lock_rhs_soa()
                            [layout.contact_vector_index(slot, component, agent)];
                    for coordinate in 0..generalized_dof {
                        contact_lock_jacobian_out[[agent, slot, component, coordinate]] =
                            self.emission_output.contact_lock_jacobian_soa()
                                [layout.contact_jacobian_index(slot, component, coordinate, agent)];
                    }
                }
            }
            status_out[agent] = self.emission_output.agent_status()[agent] as u8;
        }
        Ok(())
    }

    /// Execute the complete admitted row pipeline followed by the established
    /// strict `CpuExactF64` hierarchy. This is a CPU semantic/reference solve,
    /// not a `CpuMirrorF32` or CUDA-solver claim.
    #[allow(clippy::too_many_arguments)]
    fn run_exact_solve_batch(
        &mut self,
        py: Python<'_>,
        q: PyReadonlyArray2<'_, f32>,
        root_pose: PyReadonlyArray2<'_, f32>,
        generalized_velocity: PyReadonlyArray2<'_, f32>,
        gravity_world: PyReadonlyArray2<'_, f32>,
        point_task_active: PyReadonlyArray2<'_, u8>,
        point_target_position: PyReadonlyArray3<'_, f32>,
        point_target_velocity: PyReadonlyArray3<'_, f32>,
        point_target_acceleration: PyReadonlyArray3<'_, f32>,
        contact_lock_active: PyReadonlyArray2<'_, u8>,
        contact_desired_acceleration: PyReadonlyArray3<'_, f32>,
        acceleration_lower: PyReadonlyArray2<'_, f64>,
        acceleration_upper: PyReadonlyArray2<'_, f64>,
        mut generalized_acceleration_out: PyReadwriteArray2<'_, f64>,
        mut solve_status_out: PyReadwriteArray1<'_, u8>,
        mut level_rows_out: PyReadwriteArray2<'_, u32>,
        mut level_l2_out: PyReadwriteArray2<'_, f64>,
        mut level_rank_out: PyReadwriteArray2<'_, u32>,
        mut level_clipped_out: PyReadwriteArray2<'_, u8>,
        mut minimum_bound_margin_out: PyReadwriteArray1<'_, f64>,
        mut maximum_hard_violation_out: PyReadwriteArray1<'_, f64>,
        mut active_constraint_count_out: PyReadwriteArray1<'_, u32>,
        mut task_pseudoinverse_calls_out: PyReadwriteArray1<'_, u32>,
        mut task_jacobi_sweeps_out: PyReadwriteArray1<'_, u32>,
        mut clipped_steps_out: PyReadwriteArray1<'_, u32>,
        mut feasibility_projection_sweeps_out: PyReadwriteArray1<'_, u32>,
        mut feasibility_halfspace_projections_out: PyReadwriteArray1<'_, u32>,
        mut feasibility_polish_iterations_out: PyReadwriteArray1<'_, u32>,
        mut fk_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut jacobian_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut dynamics_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut point_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut emission_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut solve_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let q = q.as_array();
        let root_pose = root_pose.as_array();
        let generalized_velocity = generalized_velocity.as_array();
        let gravity_world = gravity_world.as_array();
        let point_task_active = point_task_active.as_array();
        let point_target_position = point_target_position.as_array();
        let point_target_velocity = point_target_velocity.as_array();
        let point_target_acceleration = point_target_acceleration.as_array();
        let contact_lock_active = contact_lock_active.as_array();
        let contact_desired_acceleration = contact_desired_acceleration.as_array();
        let acceleration_lower = acceleration_lower.as_array();
        let acceleration_upper = acceleration_upper.as_array();
        let mut generalized_acceleration_out = generalized_acceleration_out.as_array_mut();
        let solve_status_out = solve_status_out.as_slice_mut()?;
        let mut level_rows_out = level_rows_out.as_array_mut();
        let mut level_l2_out = level_l2_out.as_array_mut();
        let mut level_rank_out = level_rank_out.as_array_mut();
        let mut level_clipped_out = level_clipped_out.as_array_mut();
        let minimum_bound_margin_out = minimum_bound_margin_out.as_slice_mut()?;
        let maximum_hard_violation_out = maximum_hard_violation_out.as_slice_mut()?;
        let active_constraint_count_out = active_constraint_count_out.as_slice_mut()?;
        let task_pseudoinverse_calls_out = task_pseudoinverse_calls_out.as_slice_mut()?;
        let task_jacobi_sweeps_out = task_jacobi_sweeps_out.as_slice_mut()?;
        let clipped_steps_out = clipped_steps_out.as_slice_mut()?;
        let feasibility_projection_sweeps_out = feasibility_projection_sweeps_out.as_slice_mut()?;
        let feasibility_halfspace_projections_out =
            feasibility_halfspace_projections_out.as_slice_mut()?;
        let feasibility_polish_iterations_out = feasibility_polish_iterations_out.as_slice_mut()?;
        let fk_execute_ns_out = fk_execute_ns_out.as_slice_mut()?;
        let jacobian_execute_ns_out = jacobian_execute_ns_out.as_slice_mut()?;
        let dynamics_execute_ns_out = dynamics_execute_ns_out.as_slice_mut()?;
        let point_execute_ns_out = point_execute_ns_out.as_slice_mut()?;
        let emission_execute_ns_out = emission_execute_ns_out.as_slice_mut()?;
        let solve_execute_ns_out = solve_execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let layout = &self.executor.descriptor().layout;
        let agents = layout.agent_capacity;
        let dof = layout.coordinate_count;
        let generalized_dof = layout.generalized_coordinate_count;
        let tasks = layout.point_task_count;
        let contacts = layout.contact_lock_count;
        let stride = layout.agent_stride;
        if q.shape() != [agents, dof]
            || root_pose.shape() != [agents, POSE_COMPONENTS]
            || generalized_velocity.shape() != [agents, generalized_dof]
            || gravity_world.shape() != [agents, 3]
            || point_task_active.shape() != [agents, tasks]
            || point_target_position.shape() != [agents, tasks, 3]
            || point_target_velocity.shape() != [agents, tasks, 3]
            || point_target_acceleration.shape() != [agents, tasks, 3]
            || contact_lock_active.shape() != [agents, contacts]
            || contact_desired_acceleration.shape() != [agents, contacts, 3]
            || acceleration_lower.shape() != [agents, generalized_dof]
            || acceleration_upper.shape() != [agents, generalized_dof]
            || generalized_acceleration_out.shape() != [agents, generalized_dof]
            || solve_status_out.len() != agents
            || level_rows_out.shape() != [agents, PRIORITY_LEVEL_COUNT]
            || level_l2_out.shape() != [agents, PRIORITY_LEVEL_COUNT]
            || level_rank_out.shape() != [agents, PRIORITY_LEVEL_COUNT]
            || level_clipped_out.shape() != [agents, PRIORITY_LEVEL_COUNT]
            || minimum_bound_margin_out.len() != agents
            || maximum_hard_violation_out.len() != agents
            || active_constraint_count_out.len() != agents
            || task_pseudoinverse_calls_out.len() != agents
            || task_jacobi_sweeps_out.len() != agents
            || clipped_steps_out.len() != agents
            || feasibility_projection_sweeps_out.len() != agents
            || feasibility_halfspace_projections_out.len() != agents
            || feasibility_polish_iterations_out.len() != agents
            || fk_execute_ns_out.len() != 1
            || jacobian_execute_ns_out.len() != 1
            || dynamics_execute_ns_out.len() != 1
            || point_execute_ns_out.len() != 1
            || emission_execute_ns_out.len() != 1
            || solve_execute_ns_out.len() != 1
            || allocation_calls_out.len() != 1
            || allocated_bytes_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "CPU exact solve input/output array shape mismatch",
            ));
        }
        for agent in 0..agents {
            for coordinate in 0..dof {
                self.input.q_soa_mut()[layout.q_index(coordinate, agent)] = q[[agent, coordinate]];
            }
            for component in 0..POSE_COMPONENTS {
                self.input.root_pose_soa_mut()[layout.root_pose_index(component, agent)] =
                    root_pose[[agent, component]];
            }
            for coordinate in 0..generalized_dof {
                let index = layout.generalized_index(coordinate, agent);
                self.dynamics_input.generalized_velocity_soa_mut()[index] =
                    generalized_velocity[[agent, coordinate]];
                self.exact_solve_input.lower_soa_mut()[index] =
                    acceleration_lower[[agent, coordinate]];
                self.exact_solve_input.upper_soa_mut()[index] =
                    acceleration_upper[[agent, coordinate]];
            }
            for component in 0..3 {
                self.dynamics_input.gravity_world_soa_mut()[component * stride + agent] =
                    gravity_world[[agent, component]];
            }
            for slot in 0..tasks {
                self.emission_input.point_task_active_soa_mut()
                    [layout.task_active_index(slot, agent)] = point_task_active[[agent, slot]];
                for component in 0..3 {
                    let index = layout.task_vector_index(slot, component, agent);
                    self.emission_input.point_target_position_soa_mut()[index] =
                        point_target_position[[agent, slot, component]];
                    self.emission_input.point_target_velocity_soa_mut()[index] =
                        point_target_velocity[[agent, slot, component]];
                    self.emission_input.point_target_acceleration_soa_mut()[index] =
                        point_target_acceleration[[agent, slot, component]];
                }
            }
            for slot in 0..contacts {
                self.emission_input.contact_lock_active_soa_mut()
                    [layout.contact_active_index(slot, agent)] = contact_lock_active[[agent, slot]];
                for component in 0..3 {
                    self.emission_input.contact_desired_acceleration_soa_mut()
                        [layout.contact_vector_index(slot, component, agent)] =
                        contact_desired_acceleration[[agent, slot, component]];
                }
            }
        }
        let before = allocation_snapshot();
        let (fk_ns, jacobian_ns, dynamics_ns, point_ns, emission_ns, solve_ns) =
            py.detach(|| {
                let started = Instant::now();
                self.executor
                    .execute_into(&self.input, &mut self.output)
                    .map_err(value_error)?;
                let fk_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_jacobians_into(&self.output, &mut self.jacobian_output)
                    .map_err(value_error)?;
                let jacobian_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_dynamics_into(
                        &self.dynamics_input,
                        &self.output,
                        &self.jacobian_output,
                        &mut self.dynamics_output,
                    )
                    .map_err(value_error)?;
                let dynamics_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_point_queries_into(
                        &self.output,
                        &self.jacobian_output,
                        &self.dynamics_output,
                        &mut self.point_output,
                    )
                    .map_err(value_error)?;
                let point_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_emission_into(
                        &self.emission_input,
                        &self.dynamics_input,
                        &self.point_output,
                        &mut self.emission_output,
                    )
                    .map_err(value_error)?;
                let emission_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.exact_solver
                    .execute_into(
                        &self.emission_output,
                        &self.exact_solve_input,
                        &mut self.exact_solve_output,
                    )
                    .map_err(value_error)?;
                let solve_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                Ok::<_, PyErr>((
                    fk_ns,
                    jacobian_ns,
                    dynamics_ns,
                    point_ns,
                    emission_ns,
                    solve_ns,
                ))
            })?;
        fk_execute_ns_out[0] = fk_ns;
        jacobian_execute_ns_out[0] = jacobian_ns;
        dynamics_execute_ns_out[0] = dynamics_ns;
        point_execute_ns_out[0] = point_ns;
        emission_execute_ns_out[0] = emission_ns;
        solve_execute_ns_out[0] = solve_ns;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            for coordinate in 0..generalized_dof {
                generalized_acceleration_out[[agent, coordinate]] = self
                    .exact_solve_output
                    .generalized_acceleration_soa()[layout.generalized_index(coordinate, agent)];
            }
            solve_status_out[agent] = self.exact_solve_output.status()[agent] as u8;
            for priority in 0..PRIORITY_LEVEL_COUNT {
                let index = priority * stride + agent;
                level_rows_out[[agent, priority]] = self.exact_solve_output.level_rows_soa()[index];
                level_l2_out[[agent, priority]] = self.exact_solve_output.level_l2_soa()[index];
                level_rank_out[[agent, priority]] = self.exact_solve_output.level_rank_soa()[index];
                level_clipped_out[[agent, priority]] =
                    self.exact_solve_output.level_clipped_soa()[index];
            }
            minimum_bound_margin_out[agent] = self.exact_solve_output.minimum_bound_margin()[agent];
            maximum_hard_violation_out[agent] =
                self.exact_solve_output.maximum_hard_violation()[agent];
            active_constraint_count_out[agent] =
                self.exact_solve_output.active_constraint_count()[agent];
            task_pseudoinverse_calls_out[agent] =
                self.exact_solve_output.task_pseudoinverse_calls()[agent];
            task_jacobi_sweeps_out[agent] = self.exact_solve_output.task_jacobi_sweeps()[agent];
            clipped_steps_out[agent] = self.exact_solve_output.clipped_steps()[agent];
            feasibility_projection_sweeps_out[agent] =
                self.exact_solve_output.feasibility_projection_sweeps()[agent];
            feasibility_halfspace_projections_out[agent] =
                self.exact_solve_output.feasibility_halfspace_projections()[agent];
            feasibility_polish_iterations_out[agent] =
                self.exact_solve_output.feasibility_polish_iterations()[agent];
        }
        Ok(())
    }

    /// Run emitted point/contact semantics through the full floating dynamic
    /// CPU-exact decision vector `[qdd, generalized effort, contact force]`.
    #[allow(clippy::too_many_arguments)]
    fn run_exact_dynamic_batch(
        &mut self,
        py: Python<'_>,
        q: PyReadonlyArray2<'_, f32>,
        root_pose: PyReadonlyArray2<'_, f32>,
        generalized_velocity: PyReadonlyArray2<'_, f32>,
        gravity_world: PyReadonlyArray2<'_, f32>,
        point_task_active: PyReadonlyArray2<'_, u8>,
        point_target_position: PyReadonlyArray3<'_, f32>,
        point_target_velocity: PyReadonlyArray3<'_, f32>,
        point_target_acceleration: PyReadonlyArray3<'_, f32>,
        contact_lock_active: PyReadonlyArray2<'_, u8>,
        contact_desired_acceleration: PyReadonlyArray3<'_, f32>,
        acceleration_lower: PyReadonlyArray2<'_, f64>,
        acceleration_upper: PyReadonlyArray2<'_, f64>,
        actuator_effort_lower: PyReadonlyArray2<'_, f64>,
        actuator_effort_upper: PyReadonlyArray2<'_, f64>,
        contact_friction: PyReadonlyArray2<'_, f64>,
        contact_minimum_normal_force: PyReadonlyArray2<'_, f64>,
        contact_maximum_normal_force: PyReadonlyArray2<'_, f64>,
        contact_nominal_normal_force: PyReadonlyArray2<'_, f64>,
        joint_envelope_enabled: PyReadonlyArray1<'_, u8>,
        joint_envelope_dt_seconds: PyReadonlyArray1<'_, f64>,
        joint_maximum_acceleration: PyReadonlyArray2<'_, f64>,
        support_patch_enabled: PyReadonlyArray2<'_, u8>,
        support_patch_minimum_margin: PyReadonlyArray2<'_, f64>,
        mut generalized_acceleration_out: PyReadwriteArray2<'_, f64>,
        mut generalized_effort_out: PyReadwriteArray2<'_, f64>,
        mut contact_force_basis_out: PyReadwriteArray3<'_, f64>,
        mut contact_force_world_out: PyReadwriteArray3<'_, f64>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut active_contacts_out: PyReadwriteArray1<'_, u32>,
        mut dynamics_residual_out: PyReadwriteArray1<'_, f64>,
        mut contact_residual_out: PyReadwriteArray1<'_, f64>,
        mut minimum_friction_margin_out: PyReadwriteArray1<'_, f64>,
        mut minimum_support_margin_out: PyReadwriteArray1<'_, f64>,
        mut limiting_support_patch_out: PyReadwriteArray1<'_, u32>,
        mut active_support_patches_out: PyReadwriteArray1<'_, u32>,
        mut minimum_actuator_effort_margin_out: PyReadwriteArray1<'_, f64>,
        mut limiting_actuator_out: PyReadwriteArray1<'_, u32>,
        mut minimum_bound_margin_out: PyReadwriteArray1<'_, f64>,
        mut maximum_hard_violation_out: PyReadwriteArray1<'_, f64>,
        mut clipped_steps_out: PyReadwriteArray1<'_, u32>,
        mut task_l2_out: PyReadwriteArray2<'_, f64>,
        mut task_clipped_out: PyReadwriteArray2<'_, u8>,
        mut joint_envelope_lower_out: PyReadwriteArray2<'_, f64>,
        mut joint_envelope_upper_out: PyReadwriteArray2<'_, f64>,
        mut joint_envelope_status_out: PyReadwriteArray1<'_, u8>,
        mut joint_position_headroom_out: PyReadwriteArray1<'_, f64>,
        mut joint_velocity_headroom_out: PyReadwriteArray1<'_, f64>,
        mut limiting_joint_position_out: PyReadwriteArray1<'_, u32>,
        mut limiting_joint_velocity_out: PyReadwriteArray1<'_, u32>,
        mut joint_recovery_count_out: PyReadwriteArray1<'_, u32>,
        mut minimum_joint_envelope_margin_out: PyReadwriteArray1<'_, f64>,
        mut limiting_joint_envelope_coordinate_out: PyReadwriteArray1<'_, u32>,
        mut joint_envelope_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut fk_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut jacobian_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut dynamics_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut point_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut emission_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut solve_execute_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        let q = q.as_array();
        let root_pose = root_pose.as_array();
        let generalized_velocity = generalized_velocity.as_array();
        let gravity_world = gravity_world.as_array();
        let point_task_active = point_task_active.as_array();
        let point_target_position = point_target_position.as_array();
        let point_target_velocity = point_target_velocity.as_array();
        let point_target_acceleration = point_target_acceleration.as_array();
        let contact_lock_active = contact_lock_active.as_array();
        let contact_desired_acceleration = contact_desired_acceleration.as_array();
        let acceleration_lower = acceleration_lower.as_array();
        let acceleration_upper = acceleration_upper.as_array();
        let actuator_effort_lower = actuator_effort_lower.as_array();
        let actuator_effort_upper = actuator_effort_upper.as_array();
        let contact_friction = contact_friction.as_array();
        let contact_minimum_normal_force = contact_minimum_normal_force.as_array();
        let contact_maximum_normal_force = contact_maximum_normal_force.as_array();
        let contact_nominal_normal_force = contact_nominal_normal_force.as_array();
        let joint_envelope_enabled = joint_envelope_enabled.as_slice()?;
        let joint_envelope_dt_seconds = joint_envelope_dt_seconds.as_slice()?;
        let joint_maximum_acceleration = joint_maximum_acceleration.as_array();
        let support_patch_enabled = support_patch_enabled.as_array();
        let support_patch_minimum_margin = support_patch_minimum_margin.as_array();
        let mut generalized_acceleration_out = generalized_acceleration_out.as_array_mut();
        let mut generalized_effort_out = generalized_effort_out.as_array_mut();
        let mut contact_force_basis_out = contact_force_basis_out.as_array_mut();
        let mut contact_force_world_out = contact_force_world_out.as_array_mut();
        let status_out = status_out.as_slice_mut()?;
        let active_contacts_out = active_contacts_out.as_slice_mut()?;
        let dynamics_residual_out = dynamics_residual_out.as_slice_mut()?;
        let contact_residual_out = contact_residual_out.as_slice_mut()?;
        let minimum_friction_margin_out = minimum_friction_margin_out.as_slice_mut()?;
        let minimum_support_margin_out = minimum_support_margin_out.as_slice_mut()?;
        let limiting_support_patch_out = limiting_support_patch_out.as_slice_mut()?;
        let active_support_patches_out = active_support_patches_out.as_slice_mut()?;
        let minimum_actuator_effort_margin_out =
            minimum_actuator_effort_margin_out.as_slice_mut()?;
        let limiting_actuator_out = limiting_actuator_out.as_slice_mut()?;
        let minimum_bound_margin_out = minimum_bound_margin_out.as_slice_mut()?;
        let maximum_hard_violation_out = maximum_hard_violation_out.as_slice_mut()?;
        let clipped_steps_out = clipped_steps_out.as_slice_mut()?;
        let mut task_l2_out = task_l2_out.as_array_mut();
        let mut task_clipped_out = task_clipped_out.as_array_mut();
        let mut joint_envelope_lower_out = joint_envelope_lower_out.as_array_mut();
        let mut joint_envelope_upper_out = joint_envelope_upper_out.as_array_mut();
        let joint_envelope_status_out = joint_envelope_status_out.as_slice_mut()?;
        let joint_position_headroom_out = joint_position_headroom_out.as_slice_mut()?;
        let joint_velocity_headroom_out = joint_velocity_headroom_out.as_slice_mut()?;
        let limiting_joint_position_out = limiting_joint_position_out.as_slice_mut()?;
        let limiting_joint_velocity_out = limiting_joint_velocity_out.as_slice_mut()?;
        let joint_recovery_count_out = joint_recovery_count_out.as_slice_mut()?;
        let minimum_joint_envelope_margin_out = minimum_joint_envelope_margin_out.as_slice_mut()?;
        let limiting_joint_envelope_coordinate_out =
            limiting_joint_envelope_coordinate_out.as_slice_mut()?;
        let joint_envelope_execute_ns_out = joint_envelope_execute_ns_out.as_slice_mut()?;
        let fk_execute_ns_out = fk_execute_ns_out.as_slice_mut()?;
        let jacobian_execute_ns_out = jacobian_execute_ns_out.as_slice_mut()?;
        let dynamics_execute_ns_out = dynamics_execute_ns_out.as_slice_mut()?;
        let point_execute_ns_out = point_execute_ns_out.as_slice_mut()?;
        let emission_execute_ns_out = emission_execute_ns_out.as_slice_mut()?;
        let solve_execute_ns_out = solve_execute_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let layout = &self.executor.descriptor().layout;
        let agents = layout.agent_capacity;
        let dof = layout.coordinate_count;
        let generalized_dof = layout.generalized_coordinate_count;
        let tasks = layout.point_task_count;
        let contacts = layout.contact_lock_count;
        let support_patches = self.exact_dynamic_support_patches.len();
        let actuators = self.program.actuation.actuators.len();
        let stride = layout.agent_stride;
        let timers_valid = [
            joint_envelope_execute_ns_out.len(),
            fk_execute_ns_out.len(),
            jacobian_execute_ns_out.len(),
            dynamics_execute_ns_out.len(),
            point_execute_ns_out.len(),
            emission_execute_ns_out.len(),
            solve_execute_ns_out.len(),
            allocation_calls_out.len(),
            allocated_bytes_out.len(),
        ]
        .into_iter()
        .all(|length| length == 1);
        if q.shape() != [agents, dof]
            || root_pose.shape() != [agents, POSE_COMPONENTS]
            || generalized_velocity.shape() != [agents, generalized_dof]
            || gravity_world.shape() != [agents, 3]
            || point_task_active.shape() != [agents, tasks]
            || point_target_position.shape() != [agents, tasks, 3]
            || point_target_velocity.shape() != [agents, tasks, 3]
            || point_target_acceleration.shape() != [agents, tasks, 3]
            || contact_lock_active.shape() != [agents, contacts]
            || contact_desired_acceleration.shape() != [agents, contacts, 3]
            || acceleration_lower.shape() != [agents, generalized_dof]
            || acceleration_upper.shape() != [agents, generalized_dof]
            || actuator_effort_lower.shape() != [agents, actuators]
            || actuator_effort_upper.shape() != [agents, actuators]
            || contact_friction.shape() != [agents, contacts]
            || contact_minimum_normal_force.shape() != [agents, contacts]
            || contact_maximum_normal_force.shape() != [agents, contacts]
            || contact_nominal_normal_force.shape() != [agents, contacts]
            || joint_envelope_enabled.len() != agents
            || joint_envelope_dt_seconds.len() != agents
            || joint_maximum_acceleration.shape() != [agents, dof]
            || support_patch_enabled.shape() != [agents, support_patches]
            || support_patch_minimum_margin.shape() != [agents, support_patches]
            || generalized_acceleration_out.shape() != [agents, generalized_dof]
            || generalized_effort_out.shape() != [agents, dof]
            || contact_force_basis_out.shape() != [agents, contacts, 3]
            || contact_force_world_out.shape() != [agents, contacts, 3]
            || status_out.len() != agents
            || active_contacts_out.len() != agents
            || dynamics_residual_out.len() != agents
            || contact_residual_out.len() != agents
            || minimum_friction_margin_out.len() != agents
            || minimum_support_margin_out.len() != agents
            || limiting_support_patch_out.len() != agents
            || active_support_patches_out.len() != agents
            || minimum_actuator_effort_margin_out.len() != agents
            || limiting_actuator_out.len() != agents
            || minimum_bound_margin_out.len() != agents
            || maximum_hard_violation_out.len() != agents
            || clipped_steps_out.len() != agents
            || task_l2_out.shape() != [agents, tasks]
            || task_clipped_out.shape() != [agents, tasks]
            || joint_envelope_lower_out.shape() != [agents, generalized_dof]
            || joint_envelope_upper_out.shape() != [agents, generalized_dof]
            || joint_envelope_status_out.len() != agents
            || joint_position_headroom_out.len() != agents
            || joint_velocity_headroom_out.len() != agents
            || limiting_joint_position_out.len() != agents
            || limiting_joint_velocity_out.len() != agents
            || joint_recovery_count_out.len() != agents
            || minimum_joint_envelope_margin_out.len() != agents
            || limiting_joint_envelope_coordinate_out.len() != agents
            || !timers_valid
        {
            return Err(PyValueError::new_err(
                "CPU exact dynamic batch array shape mismatch",
            ));
        }
        for agent in 0..agents {
            self.joint_envelope_input.enabled_mut()[agent] = joint_envelope_enabled[agent];
            self.joint_envelope_input.dt_seconds_mut()[agent] = joint_envelope_dt_seconds[agent];
            for coordinate in 0..dof {
                self.input.q_soa_mut()[layout.q_index(coordinate, agent)] = q[[agent, coordinate]];
                let index = coordinate * stride + agent;
                self.joint_envelope_input.position_soa_mut()[index] = q[[agent, coordinate]] as f64;
                self.joint_envelope_input.velocity_soa_mut()[index] =
                    generalized_velocity[[agent, ROOT_TANGENT_COMPONENTS + coordinate]] as f64;
                self.joint_envelope_input.maximum_acceleration_soa_mut()[index] =
                    joint_maximum_acceleration[[agent, coordinate]];
            }
            for component in 0..POSE_COMPONENTS {
                self.input.root_pose_soa_mut()[layout.root_pose_index(component, agent)] =
                    root_pose[[agent, component]];
            }
            for coordinate in 0..generalized_dof {
                let index = layout.generalized_index(coordinate, agent);
                self.dynamics_input.generalized_velocity_soa_mut()[index] =
                    generalized_velocity[[agent, coordinate]];
                self.exact_dynamic_input.acceleration_lower_soa_mut()[index] =
                    acceleration_lower[[agent, coordinate]];
                self.exact_dynamic_input.acceleration_upper_soa_mut()[index] =
                    acceleration_upper[[agent, coordinate]];
            }
            for component in 0..3 {
                self.dynamics_input.gravity_world_soa_mut()[component * stride + agent] =
                    gravity_world[[agent, component]];
            }
            for actuator in 0..actuators {
                let index = actuator * stride + agent;
                self.exact_dynamic_input.actuator_effort_lower_soa_mut()[index] =
                    actuator_effort_lower[[agent, actuator]];
                self.exact_dynamic_input.actuator_effort_upper_soa_mut()[index] =
                    actuator_effort_upper[[agent, actuator]];
            }
            for slot in 0..tasks {
                self.emission_input.point_task_active_soa_mut()
                    [layout.task_active_index(slot, agent)] = point_task_active[[agent, slot]];
                for component in 0..3 {
                    let index = layout.task_vector_index(slot, component, agent);
                    self.emission_input.point_target_position_soa_mut()[index] =
                        point_target_position[[agent, slot, component]];
                    self.emission_input.point_target_velocity_soa_mut()[index] =
                        point_target_velocity[[agent, slot, component]];
                    self.emission_input.point_target_acceleration_soa_mut()[index] =
                        point_target_acceleration[[agent, slot, component]];
                }
            }
            for slot in 0..contacts {
                let scalar = slot * stride + agent;
                self.emission_input.contact_lock_active_soa_mut()
                    [layout.contact_active_index(slot, agent)] = contact_lock_active[[agent, slot]];
                self.exact_dynamic_input.friction_coefficient_soa_mut()[scalar] =
                    contact_friction[[agent, slot]];
                self.exact_dynamic_input.minimum_normal_force_soa_mut()[scalar] =
                    contact_minimum_normal_force[[agent, slot]];
                self.exact_dynamic_input.maximum_normal_force_soa_mut()[scalar] =
                    contact_maximum_normal_force[[agent, slot]];
                self.exact_dynamic_input.nominal_normal_force_soa_mut()[scalar] =
                    contact_nominal_normal_force[[agent, slot]];
                for component in 0..3 {
                    self.emission_input.contact_desired_acceleration_soa_mut()
                        [layout.contact_vector_index(slot, component, agent)] =
                        contact_desired_acceleration[[agent, slot, component]];
                }
            }
            for slot in 0..support_patches {
                let scalar = slot * stride + agent;
                self.exact_dynamic_input.support_patch_enabled_soa_mut()[scalar] =
                    support_patch_enabled[[agent, slot]];
                self.exact_dynamic_input
                    .support_patch_minimum_margin_soa_mut()[scalar] =
                    support_patch_minimum_margin[[agent, slot]];
            }
        }
        let before = allocation_snapshot();
        let (joint_envelope_ns, fk_ns, jacobian_ns, dynamics_ns, point_ns, emission_ns, solve_ns) =
            py.detach(|| {
                let started = Instant::now();
                self.joint_envelope_executor
                    .execute_into(&self.joint_envelope_input, &mut self.joint_envelope_output)
                    .map_err(value_error)?;
                let joint_envelope_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_into(&self.input, &mut self.output)
                    .map_err(value_error)?;
                let fk_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_jacobians_into(&self.output, &mut self.jacobian_output)
                    .map_err(value_error)?;
                let jacobian_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_dynamics_into(
                        &self.dynamics_input,
                        &self.output,
                        &self.jacobian_output,
                        &mut self.dynamics_output,
                    )
                    .map_err(value_error)?;
                let dynamics_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_point_queries_into(
                        &self.output,
                        &self.jacobian_output,
                        &self.dynamics_output,
                        &mut self.point_output,
                    )
                    .map_err(value_error)?;
                let point_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.executor
                    .execute_emission_into(
                        &self.emission_input,
                        &self.dynamics_input,
                        &self.point_output,
                        &mut self.emission_output,
                    )
                    .map_err(value_error)?;
                let emission_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                let started = Instant::now();
                self.exact_dynamic_solver
                    .execute_with_joint_envelope_into(
                        &self.input,
                        &self.dynamics_input,
                        &self.emission_input,
                        &self.emission_output,
                        &self.exact_dynamic_input,
                        &self.joint_envelope_output,
                        &mut self.exact_dynamic_output,
                    )
                    .map_err(value_error)?;
                let solve_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                Ok::<_, PyErr>((
                    joint_envelope_ns,
                    fk_ns,
                    jacobian_ns,
                    dynamics_ns,
                    point_ns,
                    emission_ns,
                    solve_ns,
                ))
            })?;
        joint_envelope_execute_ns_out[0] = joint_envelope_ns;
        fk_execute_ns_out[0] = fk_ns;
        jacobian_execute_ns_out[0] = jacobian_ns;
        dynamics_execute_ns_out[0] = dynamics_ns;
        point_execute_ns_out[0] = point_ns;
        emission_execute_ns_out[0] = emission_ns;
        solve_execute_ns_out[0] = solve_ns;
        let after = allocation_snapshot();
        allocation_calls_out[0] = after.0.saturating_sub(before.0);
        allocated_bytes_out[0] = after.1.saturating_sub(before.1);
        for agent in 0..agents {
            status_out[agent] = self.exact_dynamic_output.status()[agent] as u8;
            active_contacts_out[agent] = self.exact_dynamic_output.active_contacts()[agent];
            dynamics_residual_out[agent] =
                self.exact_dynamic_output.dynamics_residual_linf()[agent];
            contact_residual_out[agent] = self.exact_dynamic_output.contact_residual_linf()[agent];
            minimum_friction_margin_out[agent] =
                self.exact_dynamic_output.minimum_friction_margin()[agent];
            minimum_support_margin_out[agent] =
                self.exact_dynamic_output.minimum_support_margin_m()[agent];
            limiting_support_patch_out[agent] =
                self.exact_dynamic_output.limiting_support_patch()[agent];
            active_support_patches_out[agent] =
                self.exact_dynamic_output.active_support_patches()[agent];
            minimum_actuator_effort_margin_out[agent] =
                self.exact_dynamic_output.minimum_actuator_effort_margin()[agent];
            limiting_actuator_out[agent] = self.exact_dynamic_output.limiting_actuator()[agent];
            minimum_bound_margin_out[agent] =
                self.exact_dynamic_output.minimum_bound_margin()[agent];
            maximum_hard_violation_out[agent] =
                self.exact_dynamic_output.maximum_hard_violation()[agent];
            clipped_steps_out[agent] = self.exact_dynamic_output.clipped_steps()[agent];
            joint_envelope_status_out[agent] = self.joint_envelope_output.status()[agent] as u8;
            joint_position_headroom_out[agent] =
                self.joint_envelope_output.minimum_position_headroom()[agent];
            joint_velocity_headroom_out[agent] =
                self.joint_envelope_output.minimum_velocity_headroom()[agent];
            limiting_joint_position_out[agent] =
                self.joint_envelope_output.limiting_position_coordinate()[agent];
            limiting_joint_velocity_out[agent] =
                self.joint_envelope_output.limiting_velocity_coordinate()[agent];
            joint_recovery_count_out[agent] =
                self.joint_envelope_output.recovery_coordinate_count()[agent];
            minimum_joint_envelope_margin_out[agent] =
                self.exact_dynamic_output.minimum_joint_envelope_margin()[agent];
            limiting_joint_envelope_coordinate_out[agent] = self
                .exact_dynamic_output
                .limiting_joint_envelope_coordinate()[agent];
            for coordinate in 0..generalized_dof {
                generalized_acceleration_out[[agent, coordinate]] = self
                    .exact_dynamic_output
                    .generalized_acceleration_soa()[layout.generalized_index(coordinate, agent)];
                let index = layout.generalized_index(coordinate, agent);
                joint_envelope_lower_out[[agent, coordinate]] =
                    self.joint_envelope_output.lower_soa()[index];
                joint_envelope_upper_out[[agent, coordinate]] =
                    self.joint_envelope_output.upper_soa()[index];
            }
            for coordinate in 0..dof {
                generalized_effort_out[[agent, coordinate]] =
                    self.exact_dynamic_output.generalized_effort_soa()[coordinate * stride + agent];
            }
            for slot in 0..contacts {
                for component in 0..3 {
                    let index = (slot * 3 + component) * stride + agent;
                    contact_force_basis_out[[agent, slot, component]] =
                        self.exact_dynamic_output.contact_force_basis_soa()[index];
                    contact_force_world_out[[agent, slot, component]] =
                        self.exact_dynamic_output.contact_force_world_soa()[index];
                }
            }
            for slot in 0..tasks {
                task_l2_out[[agent, slot]] =
                    self.exact_dynamic_output.task_l2_soa()[slot * stride + agent];
                task_clipped_out[[agent, slot]] =
                    self.exact_dynamic_output.task_clipped_soa()[slot * stride + agent];
            }
        }
        Ok(())
    }
}

#[pymethods]
impl ActuatorResourceSession {
    /// `profiles` rows contain `[Kt, R, Rth, thermal_tau, ambient,
    /// derating_start, shutdown, minimum_effort_fraction, base_effort_limit]`.
    #[new]
    fn new(
        profiles: PyReadonlyArray2<'_, f64>,
        initial_temperature_c: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<Self> {
        let profiles = profiles.as_array();
        let initial_temperature_c = initial_temperature_c.as_slice()?;
        if profiles.ncols() != 9 || profiles.nrows() != initial_temperature_c.len() {
            return Err(PyValueError::new_err(
                "actuator resource profiles must have shape [actuators, 9]",
            ));
        }
        let mut models = Vec::with_capacity(profiles.nrows());
        let mut states = Vec::with_capacity(profiles.nrows());
        let mut base_effort_limits_nm = Vec::with_capacity(profiles.nrows());
        for actuator in 0..profiles.nrows() {
            let model = ActuatorResourceModel {
                torque_constant_nm_per_amp: profiles[[actuator, 0]],
                winding_resistance_ohm: profiles[[actuator, 1]],
                thermal_resistance_c_per_w: profiles[[actuator, 2]],
                thermal_time_constant_s: profiles[[actuator, 3]],
                ambient_temperature_c: profiles[[actuator, 4]],
                derating_start_temperature_c: profiles[[actuator, 5]],
                shutdown_temperature_c: profiles[[actuator, 6]],
                minimum_effort_fraction: profiles[[actuator, 7]],
            };
            let initial_temperature = initial_temperature_c[actuator];
            let base_effort_limit = profiles[[actuator, 8]];
            if !model.validate()
                || !initial_temperature.is_finite()
                || !base_effort_limit.is_finite()
                || base_effort_limit <= 0.0
            {
                return Err(PyValueError::new_err(format!(
                    "actuator resource profile {actuator} is invalid"
                )));
            }
            models.push(model);
            states.push(ActuatorResourceState {
                winding_temperature_c: initial_temperature,
            });
            base_effort_limits_nm.push(base_effort_limit);
        }
        Ok(Self {
            models,
            states,
            base_effort_limits_nm,
        })
    }

    #[getter]
    fn actuator_count(&self) -> usize {
        self.models.len()
    }

    #[allow(clippy::too_many_arguments)]
    fn run_trace(
        &mut self,
        actuator_effort_nm: PyReadonlyArray2<'_, f64>,
        actuator_velocity_rad_s: PyReadonlyArray2<'_, f64>,
        dt_seconds: f64,
        mut current_a_out: PyReadwriteArray2<'_, f64>,
        mut copper_loss_w_out: PyReadwriteArray2<'_, f64>,
        mut mechanical_power_w_out: PyReadwriteArray2<'_, f64>,
        mut ideal_electrical_power_w_out: PyReadwriteArray2<'_, f64>,
        mut winding_temperature_c_out: PyReadwriteArray2<'_, f64>,
        mut effort_scale_out: PyReadwriteArray2<'_, f64>,
        mut derated_effort_limit_nm_out: PyReadwriteArray2<'_, f64>,
        mut effort_headroom_nm_out: PyReadwriteArray2<'_, f64>,
        mut effort_utilization_out: PyReadwriteArray2<'_, f64>,
        mut step_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return Err(PyValueError::new_err(
                "dt_seconds must be finite and positive",
            ));
        }
        let actuator_effort_nm = actuator_effort_nm.as_array();
        let actuator_velocity_rad_s = actuator_velocity_rad_s.as_array();
        let mut current_a_out = current_a_out.as_array_mut();
        let mut copper_loss_w_out = copper_loss_w_out.as_array_mut();
        let mut mechanical_power_w_out = mechanical_power_w_out.as_array_mut();
        let mut ideal_electrical_power_w_out = ideal_electrical_power_w_out.as_array_mut();
        let mut winding_temperature_c_out = winding_temperature_c_out.as_array_mut();
        let mut effort_scale_out = effort_scale_out.as_array_mut();
        let mut derated_effort_limit_nm_out = derated_effort_limit_nm_out.as_array_mut();
        let mut effort_headroom_nm_out = effort_headroom_nm_out.as_array_mut();
        let mut effort_utilization_out = effort_utilization_out.as_array_mut();
        let step_ns_out = step_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let shape = [actuator_effort_nm.nrows(), self.models.len()];
        if actuator_effort_nm.shape() != shape
            || actuator_velocity_rad_s.shape() != shape
            || current_a_out.shape() != shape
            || copper_loss_w_out.shape() != shape
            || mechanical_power_w_out.shape() != shape
            || ideal_electrical_power_w_out.shape() != shape
            || winding_temperature_c_out.shape() != shape
            || effort_scale_out.shape() != shape
            || derated_effort_limit_nm_out.shape() != shape
            || effort_headroom_nm_out.shape() != shape
            || effort_utilization_out.shape() != shape
            || step_ns_out.len() != shape[0]
            || allocation_calls_out.len() != shape[0]
            || allocated_bytes_out.len() != shape[0]
        {
            return Err(PyValueError::new_err(
                "actuator resource trace/output shapes must agree",
            ));
        }
        for tick in 0..shape[0] {
            let before = allocation_snapshot();
            let started = Instant::now();
            for actuator in 0..shape[1] {
                let sample = step_actuator_resource(
                    self.models[actuator],
                    &mut self.states[actuator],
                    actuator_effort_nm[[tick, actuator]],
                    actuator_velocity_rad_s[[tick, actuator]],
                    self.base_effort_limits_nm[actuator],
                    dt_seconds,
                )
                .map_err(value_error)?;
                current_a_out[[tick, actuator]] = sample.current_a;
                copper_loss_w_out[[tick, actuator]] = sample.copper_loss_w;
                mechanical_power_w_out[[tick, actuator]] = sample.mechanical_power_w;
                ideal_electrical_power_w_out[[tick, actuator]] = sample.ideal_electrical_power_w;
                winding_temperature_c_out[[tick, actuator]] = sample.winding_temperature_c;
                effort_scale_out[[tick, actuator]] = sample.effort_scale;
                derated_effort_limit_nm_out[[tick, actuator]] = sample.derated_effort_limit_nm;
                effort_headroom_nm_out[[tick, actuator]] = sample.effort_headroom_nm;
                effort_utilization_out[[tick, actuator]] =
                    sample.effort_utilization.unwrap_or(f64::NAN);
            }
            let after = allocation_snapshot();
            step_ns_out[tick] = started.elapsed().as_nanos() as u64;
            allocation_calls_out[tick] = after.0 - before.0;
            allocated_bytes_out[tick] = after.1 - before.1;
        }
        Ok(())
    }
}

#[pymethods]
impl ActuatorRealizationSession {
    /// `profiles` rows contain `[effort_bandwidth_hz,
    /// maximum_effort_rate_nm_per_s]`. Positive infinity declares the exact
    /// response/no-slew control rather than an arbitrarily large finite value.
    #[new]
    fn new(
        profiles: PyReadonlyArray2<'_, f64>,
        initial_realized_effort_nm: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<Self> {
        let profiles = profiles.as_array();
        let initial_realized_effort_nm = initial_realized_effort_nm.as_slice()?;
        if profiles.ncols() != 2 || profiles.nrows() != initial_realized_effort_nm.len() {
            return Err(PyValueError::new_err(
                "actuator realization profiles must have shape [actuators, 2]",
            ));
        }
        let mut compiled_profiles = Vec::with_capacity(profiles.nrows());
        let mut states = Vec::with_capacity(profiles.nrows());
        for actuator in 0..profiles.nrows() {
            let profile = ActuatorRealizationProfile {
                effort_bandwidth_hz: profiles[[actuator, 0]],
                maximum_effort_rate_nm_per_s: profiles[[actuator, 1]],
            };
            let initial_effort = initial_realized_effort_nm[actuator];
            if !profile.validate() || !initial_effort.is_finite() {
                return Err(PyValueError::new_err(format!(
                    "actuator realization profile {actuator} is invalid"
                )));
            }
            compiled_profiles.push(profile);
            states.push(ActuatorRealizationState {
                realized_effort_nm: initial_effort,
            });
        }
        Ok(Self {
            profiles: compiled_profiles,
            states,
        })
    }

    #[getter]
    fn actuator_count(&self) -> usize {
        self.profiles.len()
    }

    /// Replace every persistent realized-effort state after validating the
    /// complete vector. This supports reset-every-sample plant experiments
    /// without reconstructing the session or partially mutating state on a
    /// late invalid coordinate.
    fn reset(&mut self, initial_realized_effort_nm: PyReadonlyArray1<'_, f64>) -> PyResult<()> {
        let initial_realized_effort_nm = initial_realized_effort_nm.as_slice()?;
        if initial_realized_effort_nm.len() != self.states.len()
            || initial_realized_effort_nm
                .iter()
                .any(|effort| !effort.is_finite())
        {
            return Err(PyValueError::new_err(
                "actuator realization reset expects one finite effort per actuator",
            ));
        }
        for (state, effort) in self
            .states
            .iter_mut()
            .zip(initial_realized_effort_nm.iter().copied())
        {
            state.realized_effort_nm = effort;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn run_trace(
        &mut self,
        requested_effort_nm: PyReadonlyArray2<'_, f64>,
        available_effort_limit_nm: PyReadonlyArray2<'_, f64>,
        dt_seconds: f64,
        mut limited_target_effort_nm_out: PyReadwriteArray2<'_, f64>,
        mut realized_effort_nm_out: PyReadwriteArray2<'_, f64>,
        mut tracking_error_nm_out: PyReadwriteArray2<'_, f64>,
        mut availability_clipped_out: PyReadwriteArray2<'_, u8>,
        mut slew_limited_out: PyReadwriteArray2<'_, u8>,
        mut step_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return Err(PyValueError::new_err(
                "dt_seconds must be finite and positive",
            ));
        }
        let requested_effort_nm = requested_effort_nm.as_array();
        let available_effort_limit_nm = available_effort_limit_nm.as_array();
        let mut limited_target_effort_nm_out = limited_target_effort_nm_out.as_array_mut();
        let mut realized_effort_nm_out = realized_effort_nm_out.as_array_mut();
        let mut tracking_error_nm_out = tracking_error_nm_out.as_array_mut();
        let mut availability_clipped_out = availability_clipped_out.as_array_mut();
        let mut slew_limited_out = slew_limited_out.as_array_mut();
        let step_ns_out = step_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let shape = [requested_effort_nm.nrows(), self.profiles.len()];
        if requested_effort_nm.shape() != shape
            || available_effort_limit_nm.shape() != shape
            || limited_target_effort_nm_out.shape() != shape
            || realized_effort_nm_out.shape() != shape
            || tracking_error_nm_out.shape() != shape
            || availability_clipped_out.shape() != shape
            || slew_limited_out.shape() != shape
            || step_ns_out.len() != shape[0]
            || allocation_calls_out.len() != shape[0]
            || allocated_bytes_out.len() != shape[0]
        {
            return Err(PyValueError::new_err(
                "actuator realization trace/output shapes must agree",
            ));
        }
        for tick in 0..shape[0] {
            let before = allocation_snapshot();
            let started = Instant::now();
            for actuator in 0..shape[1] {
                let sample = step_actuator_realization(
                    self.profiles[actuator],
                    &mut self.states[actuator],
                    requested_effort_nm[[tick, actuator]],
                    available_effort_limit_nm[[tick, actuator]],
                    dt_seconds,
                )
                .map_err(value_error)?;
                limited_target_effort_nm_out[[tick, actuator]] = sample.limited_target_effort_nm;
                realized_effort_nm_out[[tick, actuator]] = sample.realized_effort_nm;
                tracking_error_nm_out[[tick, actuator]] = sample.tracking_error_nm;
                availability_clipped_out[[tick, actuator]] = u8::from(sample.availability_clipped);
                slew_limited_out[[tick, actuator]] = u8::from(sample.slew_limited);
            }
            let after = allocation_snapshot();
            step_ns_out[tick] = started.elapsed().as_nanos() as u64;
            allocation_calls_out[tick] = after.0 - before.0;
            allocated_bytes_out[tick] = after.1 - before.1;
        }
        Ok(())
    }

    /// Advance bandwidth/slew realization and cap pointwise positive
    /// mechanical power in actuator coordinates. The full input/output shape
    /// and every input value are preflighted before persistent state or caller
    /// output can change.
    #[allow(clippy::too_many_arguments)]
    fn run_passivity_limited_trace(
        &mut self,
        requested_effort_nm: PyReadonlyArray2<'_, f64>,
        available_effort_limit_nm: PyReadonlyArray2<'_, f64>,
        actuator_velocity_rad_s: PyReadonlyArray2<'_, f64>,
        maximum_positive_mechanical_power_w: f64,
        dt_seconds: f64,
        mut limited_target_effort_nm_out: PyReadwriteArray2<'_, f64>,
        mut realized_effort_nm_out: PyReadwriteArray2<'_, f64>,
        mut tracking_error_nm_out: PyReadwriteArray2<'_, f64>,
        mut mechanical_power_w_out: PyReadwriteArray2<'_, f64>,
        mut availability_clipped_out: PyReadwriteArray2<'_, u8>,
        mut slew_limited_out: PyReadwriteArray2<'_, u8>,
        mut passivity_clipped_out: PyReadwriteArray2<'_, u8>,
        mut step_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<()> {
        if !dt_seconds.is_finite()
            || dt_seconds <= 0.0
            || !maximum_positive_mechanical_power_w.is_finite()
            || maximum_positive_mechanical_power_w < 0.0
        {
            return Err(PyValueError::new_err(
                "passivity trace requires positive finite dt and a finite nonnegative power cap",
            ));
        }
        let requested_effort_nm = requested_effort_nm.as_array();
        let available_effort_limit_nm = available_effort_limit_nm.as_array();
        let actuator_velocity_rad_s = actuator_velocity_rad_s.as_array();
        let mut limited_target_effort_nm_out = limited_target_effort_nm_out.as_array_mut();
        let mut realized_effort_nm_out = realized_effort_nm_out.as_array_mut();
        let mut tracking_error_nm_out = tracking_error_nm_out.as_array_mut();
        let mut mechanical_power_w_out = mechanical_power_w_out.as_array_mut();
        let mut availability_clipped_out = availability_clipped_out.as_array_mut();
        let mut slew_limited_out = slew_limited_out.as_array_mut();
        let mut passivity_clipped_out = passivity_clipped_out.as_array_mut();
        let step_ns_out = step_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let shape = [requested_effort_nm.nrows(), self.profiles.len()];
        if requested_effort_nm.shape() != shape
            || available_effort_limit_nm.shape() != shape
            || actuator_velocity_rad_s.shape() != shape
            || limited_target_effort_nm_out.shape() != shape
            || realized_effort_nm_out.shape() != shape
            || tracking_error_nm_out.shape() != shape
            || mechanical_power_w_out.shape() != shape
            || availability_clipped_out.shape() != shape
            || slew_limited_out.shape() != shape
            || passivity_clipped_out.shape() != shape
            || step_ns_out.len() != shape[0]
            || allocation_calls_out.len() != shape[0]
            || allocated_bytes_out.len() != shape[0]
        {
            return Err(PyValueError::new_err(
                "passivity trace/input and output shapes must agree",
            ));
        }
        if requested_effort_nm.iter().any(|value| !value.is_finite())
            || available_effort_limit_nm
                .iter()
                .any(|value| *value <= 0.0 || value.is_nan())
            || actuator_velocity_rad_s
                .iter()
                .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "passivity trace inputs must be finite with positive available effort",
            ));
        }
        for tick in 0..shape[0] {
            let before = allocation_snapshot();
            let started = Instant::now();
            for actuator in 0..shape[1] {
                let sample = step_passive_actuator_realization(
                    self.profiles[actuator],
                    &mut self.states[actuator],
                    requested_effort_nm[[tick, actuator]],
                    available_effort_limit_nm[[tick, actuator]],
                    actuator_velocity_rad_s[[tick, actuator]],
                    maximum_positive_mechanical_power_w,
                    dt_seconds,
                )
                .expect("preflighted passive actuator realization input");
                limited_target_effort_nm_out[[tick, actuator]] =
                    sample.realization.limited_target_effort_nm;
                realized_effort_nm_out[[tick, actuator]] = sample.realization.realized_effort_nm;
                tracking_error_nm_out[[tick, actuator]] = sample.realization.tracking_error_nm;
                mechanical_power_w_out[[tick, actuator]] = sample.mechanical_power_w;
                availability_clipped_out[[tick, actuator]] =
                    u8::from(sample.realization.availability_clipped);
                slew_limited_out[[tick, actuator]] = u8::from(sample.realization.slew_limited);
                passivity_clipped_out[[tick, actuator]] = u8::from(sample.passivity_clipped);
            }
            let after = allocation_snapshot();
            step_ns_out[tick] = started.elapsed().as_nanos() as u64;
            allocation_calls_out[tick] = after.0 - before.0;
            allocated_bytes_out[tick] = after.1 - before.1;
        }
        Ok(())
    }
}

#[pymethods]
impl RobotObservationHistorySession {
    #[new]
    #[pyo3(signature = (
        urdf_path,
        capacity=64,
        program_epoch=116,
        maximum_age_ns=40_000_000,
        maximum_synchronization_uncertainty_ns=2_000_000,
        maximum_interpolation_gap_ns=100_000_000,
        maximum_extrapolation_ns=40_000_000,
        maximum_source_age_ns=40_000_000,
        allow_prediction=true,
        allow_hold=false
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        urdf_path: &str,
        capacity: usize,
        program_epoch: u64,
        maximum_age_ns: i64,
        maximum_synchronization_uncertainty_ns: i64,
        maximum_interpolation_gap_ns: i64,
        maximum_extrapolation_ns: i64,
        maximum_source_age_ns: i64,
        allow_prediction: bool,
        allow_hold: bool,
    ) -> PyResult<Self> {
        if capacity == 0 {
            return Err(PyValueError::new_err("capacity must be positive"));
        }
        let limits = RobotObservationLimits {
            maximum_age_ns: Some(maximum_age_ns),
            maximum_synchronization_uncertainty_ns: Some(maximum_synchronization_uncertainty_ns),
        };
        let query_policy = RobotObservationQueryPolicy {
            maximum_interpolation_gap_ns,
            maximum_extrapolation_ns,
            maximum_source_age_ns,
            maximum_synchronization_uncertainty_ns,
            allow_prediction,
            allow_hold,
        };
        if !limits.validate() || !query_policy.validate() {
            return Err(PyValueError::new_err(
                "observation ingest and query limits must be non-negative",
            ));
        }
        let program =
            MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), program_epoch)
                .map_err(value_error)?;
        let history = RobotObservationHistory::new(&program.model, capacity);
        let output = FloatingRobotState::zeros(&program.model);
        Ok(Self {
            program,
            history,
            output,
            limits,
            query_policy,
            program_epoch,
        })
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn capacity(&self) -> usize {
        self.history.capacity()
    }

    #[getter]
    fn len(&self) -> usize {
        self.history.len()
    }

    #[getter]
    fn program_epoch(&self) -> u64 {
        self.program_epoch
    }

    fn clear(&mut self) {
        self.history.clear();
    }

    fn latest_stamp(&self) -> Option<(i64, i64, u64, u64, i64)> {
        self.history.latest_stamp().map(|stamp| {
            (
                stamp.source_time_ns,
                stamp.mapped_time_ns,
                stamp.source_sequence,
                stamp.source_id,
                stamp.synchronization_uncertainty_ns,
            )
        })
    }

    #[pyo3(signature = (
        maximum_interpolation_gap_ns,
        maximum_extrapolation_ns,
        maximum_source_age_ns,
        maximum_synchronization_uncertainty_ns,
        allow_prediction,
        allow_hold
    ))]
    fn set_query_policy(
        &mut self,
        maximum_interpolation_gap_ns: i64,
        maximum_extrapolation_ns: i64,
        maximum_source_age_ns: i64,
        maximum_synchronization_uncertainty_ns: i64,
        allow_prediction: bool,
        allow_hold: bool,
    ) -> PyResult<()> {
        let policy = RobotObservationQueryPolicy {
            maximum_interpolation_gap_ns,
            maximum_extrapolation_ns,
            maximum_source_age_ns,
            maximum_synchronization_uncertainty_ns,
            allow_prediction,
            allow_hold,
        };
        if !policy.validate() {
            return Err(PyValueError::new_err("query limits must be non-negative"));
        }
        self.query_policy = policy;
        Ok(())
    }

    /// Ingest a NumPy-authored batch. Report layout is:
    /// `[sorted, policy_valid, inserted, replaced_sequence, replaced_source,
    /// ignored_duplicate, ignored_source, rejected_epoch, rejected_future,
    /// rejected_stale, rejected_uncertain, rejected_invalid_policy,
    /// rejected_invalid_state, rejected_out_of_order, rejected_unsorted]`.
    /// Returned timing is `(elapsed_ns, allocation_calls, allocated_bytes)` and
    /// excludes NumPy decoding and construction of borrowed observation views.
    #[allow(clippy::too_many_arguments)]
    fn ingest_batch(
        &mut self,
        py: Python<'_>,
        tick_time_ns: i64,
        program_epochs: PyReadonlyArray1<'_, u64>,
        source_times_ns: PyReadonlyArray1<'_, i64>,
        mapped_times_ns: PyReadonlyArray1<'_, i64>,
        source_sequences: PyReadonlyArray1<'_, u64>,
        source_ids: PyReadonlyArray1<'_, u64>,
        synchronization_uncertainties_ns: PyReadonlyArray1<'_, i64>,
        roots_xyz_xyzw: PyReadonlyArray2<'_, f64>,
        root_twists_world: PyReadonlyArray2<'_, f64>,
        q: PyReadonlyArray2<'_, f64>,
        v: PyReadonlyArray2<'_, f64>,
        mut report_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<(u64, u64, u64)> {
        let program_epochs = program_epochs.as_slice()?;
        let source_times_ns = source_times_ns.as_slice()?;
        let mapped_times_ns = mapped_times_ns.as_slice()?;
        let source_sequences = source_sequences.as_slice()?;
        let source_ids = source_ids.as_slice()?;
        let synchronization_uncertainties_ns = synchronization_uncertainties_ns.as_slice()?;
        let roots = roots_xyz_xyzw.as_array();
        let root_twists = root_twists_world.as_array();
        let q = q.as_array();
        let v = v.as_array();
        let count = program_epochs.len();
        if source_times_ns.len() != count
            || mapped_times_ns.len() != count
            || source_sequences.len() != count
            || source_ids.len() != count
            || synchronization_uncertainties_ns.len() != count
            || roots.shape() != [count, 7]
            || root_twists.shape() != [count, 6]
            || q.shape() != [count, self.program.model.dof]
            || v.shape() != [count, self.program.model.dof]
            || report_out.len()? != 15
        {
            return Err(PyValueError::new_err(
                "observation arrays must share N; roots are Nx7, root twists are Nx6, q/v are NxDOF, report is 15",
            ));
        }

        let mut states = Vec::with_capacity(count);
        for row in 0..count {
            let quaternion_norm_squared = roots[[row, 3]].mul_add(
                roots[[row, 3]],
                roots[[row, 4]].mul_add(
                    roots[[row, 4]],
                    roots[[row, 5]].mul_add(roots[[row, 5]], roots[[row, 6]] * roots[[row, 6]]),
                ),
            );
            let rotation =
                if quaternion_norm_squared.is_finite() && quaternion_norm_squared > f64::EPSILON {
                    UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                        roots[[row, 6]],
                        roots[[row, 3]],
                        roots[[row, 4]],
                        roots[[row, 5]],
                    ))
                } else {
                    UnitQuaternion::identity()
                };
            let mut state = FloatingRobotState::zeros(&self.program.model);
            state.robot.control_world_from_root = Transform3::from_parts(
                Translation3::new(roots[[row, 0]], roots[[row, 1]], roots[[row, 2]]),
                rotation,
            );
            state.root_twist_world.0.as_mut_slice().copy_from_slice(
                root_twists
                    .row(row)
                    .as_slice()
                    .ok_or_else(|| PyValueError::new_err("root twists must be C-contiguous"))?,
            );
            state.robot.q.as_mut_slice().copy_from_slice(
                q.row(row)
                    .as_slice()
                    .ok_or_else(|| PyValueError::new_err("q must be C-contiguous"))?,
            );
            state.robot.v.as_mut_slice().copy_from_slice(
                v.row(row)
                    .as_slice()
                    .ok_or_else(|| PyValueError::new_err("v must be C-contiguous"))?,
            );
            if !quaternion_norm_squared.is_finite() || quaternion_norm_squared <= f64::EPSILON {
                state.robot.control_world_from_root.translation.x = f64::NAN;
            }
            states.push(state);
        }
        let observations: Vec<_> = states
            .iter()
            .enumerate()
            .map(|(row, state)| RobotObservationRef {
                program_epoch: program_epochs[row],
                stamp: RobotObservationStamp {
                    source_time_ns: source_times_ns[row],
                    mapped_time_ns: mapped_times_ns[row],
                    source_sequence: source_sequences[row],
                    source_id: source_ids[row],
                    synchronization_uncertainty_ns: synchronization_uncertainties_ns[row],
                },
                state,
            })
            .collect();
        let (report, elapsed_ns, allocation_calls, allocated_bytes) = py.detach(|| {
            let before = allocation_snapshot();
            let started = Instant::now();
            let report = self.history.ingest_batch(
                &self.program.model,
                tick_time_ns,
                self.program_epoch,
                self.limits,
                &observations,
            );
            let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let after = allocation_snapshot();
            (
                report,
                elapsed_ns,
                after.0.saturating_sub(before.0),
                after.1.saturating_sub(before.1),
            )
        });
        let report_out = report_out.as_slice_mut()?;
        report_out.copy_from_slice(&[
            report.batch_sorted as u64,
            report.policy_valid as u64,
            report.inserted as u64,
            report.replaced_sequence as u64,
            report.replaced_source as u64,
            report.ignored_duplicate as u64,
            report.ignored_source as u64,
            report.rejected_epoch as u64,
            report.rejected_future as u64,
            report.rejected_stale as u64,
            report.rejected_uncertain as u64,
            report.rejected_invalid_policy as u64,
            report.rejected_invalid_state as u64,
            report.rejected_out_of_order as u64,
            report.rejected_unsorted as u64,
        ]);
        Ok((elapsed_ns, allocation_calls, allocated_bytes))
    }

    /// Evidence layout is `[provenance, program_epoch, lower_time, upper_time,
    /// lower_source, upper_source, lower_sequence, upper_sequence, source_age,
    /// age_headroom, max_sync_uncertainty, sync_headroom, hard_eligible,
    /// lower_source_time, upper_source_time]`. The status code is zero on
    /// success, then Empty/BeforeHistory/Gap/Extrapolation/TooOld/Sync/Invalid
    /// policy or reconstructed state.
    fn reconstruct_into(
        &mut self,
        py: Python<'_>,
        time_ns: i64,
        mut root_xyz_xyzw_out: PyReadwriteArray1<'_, f64>,
        mut root_twist_world_out: PyReadwriteArray1<'_, f64>,
        mut q_out: PyReadwriteArray1<'_, f64>,
        mut v_out: PyReadwriteArray1<'_, f64>,
        mut evidence_out: PyReadwriteArray1<'_, i64>,
    ) -> PyResult<(u8, u64, u64, u64)> {
        if root_xyz_xyzw_out.len()? != 7
            || root_twist_world_out.len()? != 6
            || q_out.len()? != self.program.model.dof
            || v_out.len()? != self.program.model.dof
            || evidence_out.len()? != 15
        {
            return Err(PyValueError::new_err(
                "root output is 7, root twist output is 6, q/v outputs are DOF, and evidence output is 15",
            ));
        }
        let (result, elapsed_ns, allocation_calls, allocated_bytes) = py.detach(|| {
            let before = allocation_snapshot();
            let started = Instant::now();
            let result = self.history.reconstruct_into(
                &self.program.model,
                time_ns,
                self.query_policy,
                &mut self.output,
            );
            let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let after = allocation_snapshot();
            (
                result,
                elapsed_ns,
                after.0.saturating_sub(before.0),
                after.1.saturating_sub(before.1),
            )
        });
        let status = match result {
            Ok(evidence) => {
                let translation = self.output.robot.control_world_from_root.translation.vector;
                let quaternion = self
                    .output
                    .robot
                    .control_world_from_root
                    .rotation
                    .quaternion();
                root_xyz_xyzw_out.as_slice_mut()?.copy_from_slice(&[
                    translation.x,
                    translation.y,
                    translation.z,
                    quaternion.i,
                    quaternion.j,
                    quaternion.k,
                    quaternion.w,
                ]);
                root_twist_world_out
                    .as_slice_mut()?
                    .copy_from_slice(self.output.root_twist_world.0.as_slice());
                q_out
                    .as_slice_mut()?
                    .copy_from_slice(self.output.robot.q.as_slice());
                v_out
                    .as_slice_mut()?
                    .copy_from_slice(self.output.robot.v.as_slice());
                evidence_out.as_slice_mut()?.copy_from_slice(&[
                    reconstruction_provenance_code(evidence.provenance),
                    evidence.program_epoch as i64,
                    evidence.source_interval_ns.0,
                    evidence.source_interval_ns.1,
                    evidence.lower_stamp.source_id as i64,
                    evidence.upper_stamp.source_id as i64,
                    evidence.lower_stamp.source_sequence as i64,
                    evidence.upper_stamp.source_sequence as i64,
                    evidence.source_age_ns,
                    evidence.source_age_headroom_ns,
                    evidence.maximum_synchronization_uncertainty_ns,
                    evidence.synchronization_headroom_ns,
                    evidence.hard_constraint_eligible as i64,
                    evidence.lower_stamp.source_time_ns,
                    evidence.upper_stamp.source_time_ns,
                ]);
                0
            }
            Err(error) => robot_observation_query_error_code(error),
        };
        Ok((status, elapsed_ns, allocation_calls, allocated_bytes))
    }
}

fn reconstruction_provenance_code(provenance: ReconstructionProvenance) -> i64 {
    match provenance {
        ReconstructionProvenance::ExactSample => 0,
        ReconstructionProvenance::Interpolated => 1,
        ReconstructionProvenance::PredictedConstantVelocity => 2,
        ReconstructionProvenance::Held => 3,
    }
}

fn robot_observation_query_error_code(error: RobotObservationQueryError) -> u8 {
    match error {
        RobotObservationQueryError::Empty => 1,
        RobotObservationQueryError::BeforeHistory => 2,
        RobotObservationQueryError::InterpolationGap => 3,
        RobotObservationQueryError::Extrapolation => 4,
        RobotObservationQueryError::TooOld => 5,
        RobotObservationQueryError::SynchronizationUncertain => 6,
        RobotObservationQueryError::InvalidPolicyOrLayout => 7,
        RobotObservationQueryError::ReconstructedStateInvalid => 8,
    }
}

#[pymethods]
impl CollisionAvoidanceSession {
    #[new]
    fn new(urdf_path: &str) -> PyResult<Self> {
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let collision = CompiledCollisionModel::compile(&program.model);
        let state = RobotState::zeros(&program.model);
        let cache = ModelCache::new(&program.model);
        let sample = DistanceSample::workspace(program.model.dof);
        let scratch = CollisionEvaluationScratch::new(
            program.model.dof,
            collision.validation_primitives.len(),
            collision.validation_pairs.len(),
        );
        Ok(Self {
            program,
            collision,
            state,
            cache,
            sample,
            scratch,
        })
    }

    #[getter]
    fn model_name(&self) -> &str {
        &self.program.model.name
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn tight_pair_count(&self) -> usize {
        self.collision.validation_pairs.len()
    }

    #[getter]
    fn sphere_cover_pair_count(&self) -> usize {
        self.collision.self_pairs.len()
    }

    /// Evaluate either the shared tight-primitive table (`tight=true`) or the
    /// explicit legacy sphere-cover fallback into caller-owned NumPy buffers.
    /// The returned tuple is `(elapsed_ns, allocation_calls, allocated_bytes)`
    /// for state copy, FK, closest-feature distance, and Jacobian evaluation.
    #[allow(clippy::too_many_arguments)]
    fn evaluate_into(
        &mut self,
        tight: bool,
        q: PyReadonlyArray1<'_, f64>,
        v: PyReadonlyArray1<'_, f64>,
        mut pair_ids_out: PyReadwriteArray1<'_, u32>,
        mut quality_out: PyReadwriteArray1<'_, u8>,
        mut signed_distance_out: PyReadwriteArray1<'_, f64>,
        mut normal_out: PyReadwriteArray2<'_, f64>,
        mut point_a_out: PyReadwriteArray2<'_, f64>,
        mut point_b_out: PyReadwriteArray2<'_, f64>,
        mut jacobian_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let q = q.as_slice()?;
        let v = v.as_slice()?;
        let dof = self.program.model.dof;
        let pair_count = if tight {
            self.collision.validation_pairs.len()
        } else {
            self.collision.self_pairs.len()
        };
        let mut pair_ids_out = pair_ids_out.as_array_mut();
        let mut quality_out = quality_out.as_array_mut();
        let mut signed_distance_out = signed_distance_out.as_array_mut();
        let mut normal_out = normal_out.as_array_mut();
        let mut point_a_out = point_a_out.as_array_mut();
        let mut point_b_out = point_b_out.as_array_mut();
        let mut jacobian_out = jacobian_out.as_array_mut();
        if q.len() != dof
            || v.len() != dof
            || pair_ids_out.len() != pair_count
            || quality_out.len() != pair_count
            || signed_distance_out.len() != pair_count
            || normal_out.shape() != [pair_count, 3]
            || point_a_out.shape() != [pair_count, 3]
            || point_b_out.shape() != [pair_count, 3]
            || jacobian_out.shape() != [pair_count, dof]
        {
            return Err(PyValueError::new_err(
                "collision evaluation input/output dimension mismatch",
            ));
        }

        let before = allocation_snapshot();
        let started = Instant::now();
        self.state.q.as_mut_slice().copy_from_slice(q);
        self.state.v.as_mut_slice().copy_from_slice(v);
        self.state
            .validate(&self.program.model)
            .map_err(value_error)?;
        self.program
            .model
            .forward_kinematics(&self.state, &mut self.cache)
            .map_err(value_error)?;
        for pair_index in 0..pair_count {
            let pair = if tight {
                self.collision.validation_pairs[pair_index]
            } else {
                self.collision.self_pairs[pair_index]
            };
            if tight {
                self.collision
                    .evaluate_validation_pair_into(
                        &self.program.model,
                        &self.cache,
                        &self.state.v,
                        pair,
                        &mut self.sample,
                        &mut self.scratch,
                    )
                    .map_err(value_error)?;
            } else {
                self.collision
                    .evaluate_pair_into(
                        &self.program.model,
                        &self.cache,
                        &self.state.v,
                        pair,
                        &mut self.sample,
                        &mut self.scratch,
                    )
                    .map_err(value_error)?;
            }
            pair_ids_out[pair_index] = self.sample.pair_id;
            quality_out[pair_index] = match self.sample.quality {
                DistanceQuality::ExactSphere => 0,
                DistanceQuality::AnalyticPrimitive => 1,
                DistanceQuality::ConservativePrimitive => 2,
                DistanceQuality::ConservativeBoundingSphere => 3,
                DistanceQuality::ConservativeSphereCover => 4,
            };
            signed_distance_out[pair_index] = self.sample.signed_distance;
            for axis in 0..3 {
                normal_out[[pair_index, axis]] = self.sample.normal_in_control_world[axis];
                point_a_out[[pair_index, axis]] = self.sample.point_a_in_control_world[axis];
                point_b_out[[pair_index, axis]] = self.sample.point_b_in_control_world[axis];
            }
            for coordinate in 0..dof {
                jacobian_out[[pair_index, coordinate]] = self.sample.jacobian_row[coordinate];
            }
        }
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let after = allocation_snapshot();
        Ok((elapsed_ns, after.0 - before.0, after.1 - before.1))
    }
}

#[pymethods]
impl FloatingCollisionBarrierSession {
    #[new]
    #[pyo3(signature = (
        urdf_path,
        enabled=true,
        hard_margin_m=0.02,
        influence_margin_m=0.10,
        natural_frequency_hz=2.0,
        damping_ratio=1.0,
        maximum_acceleration=200.0,
        maximum_torque=2000.0
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        urdf_path: &str,
        enabled: bool,
        hard_margin_m: f64,
        influence_margin_m: f64,
        natural_frequency_hz: f64,
        damping_ratio: f64,
        maximum_acceleration: f64,
        maximum_torque: f64,
    ) -> PyResult<Self> {
        if !natural_frequency_hz.is_finite()
            || natural_frequency_hz <= 0.0
            || !maximum_acceleration.is_finite()
            || maximum_acceleration <= 0.0
            || !maximum_torque.is_finite()
            || maximum_torque <= 0.0
        {
            return Err(PyValueError::new_err(
                "floating collision barrier limits/frequency must be finite and positive",
            ));
        }
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let barrier = CollisionAccelerationBarrierConfig {
            hard_margin: hard_margin_m,
            influence_margin: influence_margin_m,
            natural_frequency_rad_s: std::f64::consts::TAU * natural_frequency_hz,
            damping_ratio,
            ..CollisionAccelerationBarrierConfig::default()
        };
        let controller = FloatingDynamicWbc::new(
            program.model.clone(),
            DynamicWbcConfig {
                gravity_world: Vec3::zeros(),
                floating_collision_barrier: enabled.then_some(barrier),
                ..DynamicWbcConfig::default()
            },
        )
        .map_err(value_error)?;
        let dof = program.model.dof;
        let generalized_dof = dof + 6;
        let scratch = FloatingDynamicWbcScratch::new(&program.model, 0);
        let output =
            FloatingDynamicWbcOutput::workspace(dof, 0, controller.maximum_constraint_count(0, 0));
        Ok(Self {
            state: FloatingRobotState::zeros(&program.model),
            scratch,
            output,
            desired_acceleration: DVector::zeros(generalized_dof),
            acceleration_bounds: VelocityBounds {
                lower: DVector::from_element(generalized_dof, -maximum_acceleration),
                upper: DVector::from_element(generalized_dof, maximum_acceleration),
            },
            torque_bounds: VelocityBounds {
                lower: DVector::from_element(dof, -maximum_torque),
                upper: DVector::from_element(dof, maximum_torque),
            },
            program,
            controller,
        })
    }

    #[getter]
    fn model_name(&self) -> &str {
        &self.program.model.name
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn pair_count(&self) -> usize {
        self.controller.collision_pair_count()
    }

    /// Execute one caller-authored floating state query. Evidence layout is:
    /// `[distance, margin, relative_velocity, bias_acceleration,
    /// required_acceleration, achieved_acceleration, barrier_residual]`.
    /// IDs are `[closest_pair, limiting_active_pair]`; counts are
    /// `[represented_pairs, unsupported_shapes, active_pairs]`.
    #[allow(clippy::too_many_arguments)]
    fn evaluate_into(
        &mut self,
        q: PyReadonlyArray1<'_, f64>,
        v: PyReadonlyArray1<'_, f64>,
        root_translation: PyReadonlyArray1<'_, f64>,
        root_twist: PyReadonlyArray1<'_, f64>,
        desired_generalized_acceleration: PyReadonlyArray1<'_, f64>,
        mut generalized_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut evidence_out: PyReadwriteArray1<'_, f64>,
        mut pair_ids_out: PyReadwriteArray1<'_, u32>,
        mut counts_out: PyReadwriteArray1<'_, u32>,
        mut quality_out: PyReadwriteArray1<'_, u8>,
        mut status_out: PyReadwriteArray1<'_, u8>,
    ) -> PyResult<(u64, u64, u64)> {
        let q = q.as_slice()?;
        let v = v.as_slice()?;
        let root_translation = root_translation.as_slice()?;
        let root_twist = root_twist.as_slice()?;
        let desired = desired_generalized_acceleration.as_slice()?;
        let generalized_acceleration_out = generalized_acceleration_out.as_slice_mut()?;
        let evidence_out = evidence_out.as_slice_mut()?;
        let pair_ids_out = pair_ids_out.as_slice_mut()?;
        let counts_out = counts_out.as_slice_mut()?;
        let quality_out = quality_out.as_slice_mut()?;
        let status_out = status_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if q.len() != dof
            || v.len() != dof
            || root_translation.len() != 3
            || root_twist.len() != 6
            || desired.len() != generalized_dof
            || generalized_acceleration_out.len() != generalized_dof
            || evidence_out.len() != 7
            || pair_ids_out.len() != 2
            || counts_out.len() != 3
            || quality_out.len() != 2
            || status_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "floating collision barrier input/output dimension mismatch",
            ));
        }

        let before = allocation_snapshot();
        let started = Instant::now();
        self.state.robot.q.as_mut_slice().copy_from_slice(q);
        self.state.robot.v.as_mut_slice().copy_from_slice(v);
        self.state
            .robot
            .control_world_from_root
            .translation
            .vector
            .as_mut_slice()
            .copy_from_slice(root_translation);
        self.state
            .root_twist_world
            .0
            .as_mut_slice()
            .copy_from_slice(root_twist);
        self.desired_acceleration
            .as_mut_slice()
            .copy_from_slice(desired);
        self.controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &self.state.robot,
                    root_twist_world: self.state.root_twist_world,
                    desired_generalized_acceleration: &self.desired_acceleration,
                    task_priorities: FloatingTaskPriorities::default(),
                    task_weights: FloatingTaskWeights::default(),
                    joint_posture_weight: 1.0,
                    joint_acceleration_task: None,
                    center_of_mass_task: None,
                    centroidal_angular_momentum_task: None,
                    frame_angular_acceleration_tasks: &[],
                    point_acceleration_tasks: &[],
                    generalized_acceleration_bounds: &self.acceleration_bounds,
                    torque_bounds: &self.torque_bounds,
                    actuator_effort: None,
                    contacts: &[],
                    support_patches: &[],
                },
                &mut self.output,
                &mut self.scratch,
            )
            .map_err(value_error)?;
        generalized_acceleration_out
            .copy_from_slice(self.output.generalized_acceleration.as_slice());
        let evidence = self.output.collision_barrier;
        evidence_out.copy_from_slice(&[
            evidence.minimum_signed_distance_m,
            evidence.minimum_margin_m,
            evidence.limiting_relative_normal_velocity_mps,
            evidence.limiting_normal_bias_acceleration_mps2,
            evidence.limiting_required_normal_acceleration_mps2,
            evidence.limiting_achieved_normal_acceleration_mps2,
            evidence.minimum_barrier_residual_mps2,
        ]);
        pair_ids_out[0] = evidence.closest_pair_id.unwrap_or(u32::MAX);
        pair_ids_out[1] = evidence.limiting_active_pair_id.unwrap_or(u32::MAX);
        counts_out[0] = evidence.represented_pair_count.min(u32::MAX as usize) as u32;
        counts_out[1] = evidence.unsupported_shape_count.min(u32::MAX as usize) as u32;
        counts_out[2] = evidence.active_pair_count.min(u32::MAX as usize) as u32;
        let quality_code = |quality: Option<DistanceQuality>| match quality {
            Some(DistanceQuality::ExactSphere) => 0,
            Some(DistanceQuality::AnalyticPrimitive) => 1,
            Some(DistanceQuality::ConservativePrimitive) => 2,
            Some(DistanceQuality::ConservativeBoundingSphere) => 3,
            Some(DistanceQuality::ConservativeSphereCover) => 4,
            None => u8::MAX,
        };
        quality_out[0] = quality_code(evidence.closest_quality);
        quality_out[1] = quality_code(evidence.limiting_active_quality);
        status_out[0] = match self.output.status {
            SolveStatus::Solved => 0,
            SolveStatus::SolvedWithSlack => 1,
            SolveStatus::PrimalInfeasible => 2,
            SolveStatus::NumericalFailure | SolveStatus::InvalidProblem => 3,
            SolveStatus::MaxIterations => 4,
        };
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let after = allocation_snapshot();
        Ok((elapsed_ns, after.0 - before.0, after.1 - before.1))
    }
}

#[pymethods]
impl WorldSdfBarrierSession {
    #[new]
    #[pyo3(signature = (
        urdf_path,
        sdf_values_zyx,
        grid_origin_world,
        grid_spacing,
        outside_policy=0,
        enabled=true,
        hard_margin_m=0.02,
        influence_margin_m=0.10,
        natural_frequency_hz=2.0,
        damping_ratio=1.0,
        maximum_acceleration=200.0,
        maximum_torque=2000.0
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        urdf_path: &str,
        sdf_values_zyx: PyReadonlyArray3<'_, f64>,
        grid_origin_world: PyReadonlyArray1<'_, f64>,
        grid_spacing: PyReadonlyArray1<'_, f64>,
        outside_policy: u8,
        enabled: bool,
        hard_margin_m: f64,
        influence_margin_m: f64,
        natural_frequency_hz: f64,
        damping_ratio: f64,
        maximum_acceleration: f64,
        maximum_torque: f64,
    ) -> PyResult<Self> {
        let origin = grid_origin_world.as_slice()?;
        let spacing = grid_spacing.as_slice()?;
        let values = sdf_values_zyx.as_array();
        let shape = values.shape();
        if origin.len() != 3
            || spacing.len() != 3
            || shape.len() != 3
            || shape.iter().any(|dimension| *dimension < 2)
            || !natural_frequency_hz.is_finite()
            || natural_frequency_hz <= 0.0
            || !maximum_acceleration.is_finite()
            || maximum_acceleration <= 0.0
            || !maximum_torque.is_finite()
            || maximum_torque <= 0.0
        {
            return Err(PyValueError::new_err(
                "world SDF layout, limits, or frequency are invalid",
            ));
        }
        let outside_policy = match outside_policy {
            0 => SdfOutsidePolicy::Reject,
            1 => SdfOutsidePolicy::OccupiedBoundary,
            _ => {
                return Err(PyValueError::new_err(
                    "outside_policy must be 0 (reject) or 1 (occupied boundary)",
                ));
            }
        };
        let field = DenseSdfGrid::new(
            bonesaw_core::Transform3::from_parts(
                Translation3::new(origin[0], origin[1], origin[2]),
                UnitQuaternion::identity(),
            ),
            [shape[2], shape[1], shape[0]],
            Vec3::new(spacing[0], spacing[1], spacing[2]),
            values.iter().copied().collect(),
            outside_policy,
        )
        .map_err(value_error)?;
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let world_collision =
            CompiledWorldCollisionModel::compile(&program.model, field).map_err(value_error)?;
        let barrier = CollisionAccelerationBarrierConfig {
            hard_margin: hard_margin_m,
            influence_margin: influence_margin_m,
            natural_frequency_rad_s: std::f64::consts::TAU * natural_frequency_hz,
            damping_ratio,
            hard_stable_id_base: 0x7000_0000,
        };
        let controller = if enabled {
            FloatingDynamicWbc::new_with_world_collision(
                program.model.clone(),
                DynamicWbcConfig {
                    gravity_world: Vec3::zeros(),
                    floating_world_collision_barrier: Some(barrier),
                    ..DynamicWbcConfig::default()
                },
                world_collision,
            )
        } else {
            FloatingDynamicWbc::new(
                program.model.clone(),
                DynamicWbcConfig {
                    gravity_world: Vec3::zeros(),
                    ..DynamicWbcConfig::default()
                },
            )
        }
        .map_err(value_error)?;
        let dof = program.model.dof;
        let generalized_dof = dof + 6;
        let scratch = controller.scratch(0, 0);
        let output =
            FloatingDynamicWbcOutput::workspace(dof, 0, controller.maximum_constraint_count(0, 0));
        Ok(Self {
            state: FloatingRobotState::zeros(&program.model),
            scratch,
            output,
            desired_acceleration: DVector::zeros(generalized_dof),
            acceleration_bounds: VelocityBounds {
                lower: DVector::from_element(generalized_dof, -maximum_acceleration),
                upper: DVector::from_element(generalized_dof, maximum_acceleration),
            },
            torque_bounds: VelocityBounds {
                lower: DVector::from_element(dof, -maximum_torque),
                upper: DVector::from_element(dof, maximum_torque),
            },
            program,
            controller,
        })
    }

    #[getter]
    fn model_name(&self) -> &str {
        &self.program.model.name
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn probe_count(&self) -> usize {
        self.controller.world_collision_probe_count()
    }

    /// Evidence layout is `[distance, margin, closest_gradient_norm,
    /// relative_velocity, bias_acceleration, required_acceleration,
    /// achieved_acceleration, barrier_residual]`. IDs and bodies are ordered
    /// `[closest, limiting_active]`; counts are `[represented, unsupported,
    /// active]`; source codes are 0=trilinear, 1=occupied boundary.
    #[allow(clippy::too_many_arguments)]
    fn evaluate_into(
        &mut self,
        q: PyReadonlyArray1<'_, f64>,
        v: PyReadonlyArray1<'_, f64>,
        root_translation: PyReadonlyArray1<'_, f64>,
        root_twist: PyReadonlyArray1<'_, f64>,
        desired_generalized_acceleration: PyReadonlyArray1<'_, f64>,
        mut generalized_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut evidence_out: PyReadwriteArray1<'_, f64>,
        mut probe_ids_out: PyReadwriteArray1<'_, u32>,
        mut body_ids_out: PyReadwriteArray1<'_, u32>,
        mut counts_out: PyReadwriteArray1<'_, u32>,
        mut source_out: PyReadwriteArray1<'_, u8>,
        mut proxy_quality_out: PyReadwriteArray1<'_, u8>,
        mut status_out: PyReadwriteArray1<'_, u8>,
    ) -> PyResult<(u64, u64, u64)> {
        let q = q.as_slice()?;
        let v = v.as_slice()?;
        let root_translation = root_translation.as_slice()?;
        let root_twist = root_twist.as_slice()?;
        let desired = desired_generalized_acceleration.as_slice()?;
        let generalized_acceleration_out = generalized_acceleration_out.as_slice_mut()?;
        let evidence_out = evidence_out.as_slice_mut()?;
        let probe_ids_out = probe_ids_out.as_slice_mut()?;
        let body_ids_out = body_ids_out.as_slice_mut()?;
        let counts_out = counts_out.as_slice_mut()?;
        let source_out = source_out.as_slice_mut()?;
        let proxy_quality_out = proxy_quality_out.as_slice_mut()?;
        let status_out = status_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if q.len() != dof
            || v.len() != dof
            || root_translation.len() != 3
            || root_twist.len() != 6
            || desired.len() != generalized_dof
            || generalized_acceleration_out.len() != generalized_dof
            || evidence_out.len() != 8
            || probe_ids_out.len() != 2
            || body_ids_out.len() != 2
            || counts_out.len() != 3
            || source_out.len() != 2
            || proxy_quality_out.len() != 2
            || status_out.len() != 1
        {
            return Err(PyValueError::new_err(
                "world SDF barrier input/output dimension mismatch",
            ));
        }

        let before = allocation_snapshot();
        let started = Instant::now();
        self.state.robot.q.as_mut_slice().copy_from_slice(q);
        self.state.robot.v.as_mut_slice().copy_from_slice(v);
        self.state
            .robot
            .control_world_from_root
            .translation
            .vector
            .as_mut_slice()
            .copy_from_slice(root_translation);
        self.state
            .root_twist_world
            .0
            .as_mut_slice()
            .copy_from_slice(root_twist);
        self.desired_acceleration
            .as_mut_slice()
            .copy_from_slice(desired);
        self.controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &self.state.robot,
                    root_twist_world: self.state.root_twist_world,
                    desired_generalized_acceleration: &self.desired_acceleration,
                    task_priorities: FloatingTaskPriorities::default(),
                    task_weights: FloatingTaskWeights::default(),
                    joint_posture_weight: 1.0,
                    joint_acceleration_task: None,
                    center_of_mass_task: None,
                    centroidal_angular_momentum_task: None,
                    frame_angular_acceleration_tasks: &[],
                    point_acceleration_tasks: &[],
                    generalized_acceleration_bounds: &self.acceleration_bounds,
                    torque_bounds: &self.torque_bounds,
                    actuator_effort: None,
                    contacts: &[],
                    support_patches: &[],
                },
                &mut self.output,
                &mut self.scratch,
            )
            .map_err(value_error)?;
        generalized_acceleration_out
            .copy_from_slice(self.output.generalized_acceleration.as_slice());
        let evidence = self.output.world_collision_barrier;
        evidence_out.copy_from_slice(&[
            evidence.minimum_signed_distance_m,
            evidence.minimum_margin_m,
            evidence.closest_gradient_norm,
            evidence.limiting_relative_normal_velocity_mps,
            evidence.limiting_normal_bias_acceleration_mps2,
            evidence.limiting_required_normal_acceleration_mps2,
            evidence.limiting_achieved_normal_acceleration_mps2,
            evidence.minimum_barrier_residual_mps2,
        ]);
        probe_ids_out[0] = evidence.closest_probe_id.unwrap_or(u32::MAX);
        probe_ids_out[1] = evidence.limiting_active_probe_id.unwrap_or(u32::MAX);
        body_ids_out[0] = evidence
            .closest_body
            .map_or(u32::MAX, |body| body.0.min(u32::MAX as usize) as u32);
        body_ids_out[1] = evidence
            .limiting_active_body
            .map_or(u32::MAX, |body| body.0.min(u32::MAX as usize) as u32);
        counts_out[0] = evidence.represented_probe_count.min(u32::MAX as usize) as u32;
        counts_out[1] = evidence.unsupported_shape_count.min(u32::MAX as usize) as u32;
        counts_out[2] = evidence.active_probe_count.min(u32::MAX as usize) as u32;
        let source_code = |source: Option<SdfSampleSource>| match source {
            Some(SdfSampleSource::TrilinearGrid) => 0,
            Some(SdfSampleSource::OccupiedBoundary) => 1,
            None => u8::MAX,
        };
        source_out[0] = source_code(evidence.closest_field_source);
        source_out[1] = source_code(evidence.limiting_field_source);
        let quality_code = |quality: Option<DistanceQuality>| match quality {
            Some(DistanceQuality::ExactSphere) => 0,
            Some(DistanceQuality::AnalyticPrimitive) => 1,
            Some(DistanceQuality::ConservativePrimitive) => 2,
            Some(DistanceQuality::ConservativeBoundingSphere) => 3,
            Some(DistanceQuality::ConservativeSphereCover) => 4,
            None => u8::MAX,
        };
        proxy_quality_out[0] = quality_code(evidence.closest_proxy_quality);
        proxy_quality_out[1] = quality_code(evidence.limiting_proxy_quality);
        status_out[0] = match self.output.status {
            SolveStatus::Solved => 0,
            SolveStatus::SolvedWithSlack => 1,
            SolveStatus::PrimalInfeasible => 2,
            SolveStatus::NumericalFailure | SolveStatus::InvalidProblem => 3,
            SolveStatus::MaxIterations => 4,
        };
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let after = allocation_snapshot();
        Ok((elapsed_ns, after.0 - before.0, after.1 - before.1))
    }
}

#[pymethods]
impl ControllerSession {
    #[new]
    #[pyo3(signature = (urdf_path, task_capacity=32))]
    fn new(urdf_path: &str, task_capacity: usize) -> PyResult<Self> {
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let controller = Controller::from_program(&program).map_err(value_error)?;
        let initial = RobotState::zeros(&program.model);
        let state_in = ControllerState::for_program(initial.clone(), &program);
        let state_out = state_in.clone();
        let scratch = ControllerScratch::for_program(&program, task_capacity);
        let output = ControllerOutputBuffer::with_layout(
            &program.model,
            program.timing.control_horizon_ns,
            program.timing.sample_period_ns,
        );
        let input = ControllerInput {
            tick_time_ns: 0,
            state: initial,
            signal_inputs: Default::default(),
            frame_targets: Vec::with_capacity(task_capacity),
            com_target_world: None,
            posture_target: Some(DVector::zeros(program.model.dof)),
            hard_constraints: Vec::new(),
        };
        Ok(Self {
            program,
            controller,
            state_in,
            state_out,
            scratch,
            output,
            input,
        })
    }

    #[getter]
    fn model_name(&self) -> &str {
        &self.program.model.name
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn bodies(&self) -> usize {
        self.program.model.bodies.len()
    }

    #[getter]
    fn frame_names(&self) -> Vec<&str> {
        self.program
            .model
            .bodies
            .iter()
            .map(|body| body.name.as_str())
            .collect()
    }

    #[getter]
    fn joint_names(&self) -> Vec<&str> {
        self.program
            .model
            .joints
            .iter()
            .filter(|joint| joint.coordinate.is_some())
            .map(|joint| joint.name.as_str())
            .collect()
    }

    fn reset(
        &mut self,
        q: PyReadonlyArray1<'_, f64>,
        v: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        let q = q.as_slice()?;
        let v = v.as_slice()?;
        if q.len() != self.dof() || v.len() != self.dof() {
            return Err(PyValueError::new_err("reset q/v dimension mismatch"));
        }
        let mut initial = RobotState::zeros(&self.program.model);
        initial.q.as_mut_slice().copy_from_slice(q);
        initial.v.as_mut_slice().copy_from_slice(v);
        initial.validate(&self.program.model).map_err(value_error)?;
        self.state_in = ControllerState::for_program(initial.clone(), &self.program);
        self.state_out = ControllerState::for_program(initial.clone(), &self.program);
        self.input.state = initial;
        Ok(())
    }

    fn frame_positions(
        &self,
        q: PyReadonlyArray1<'_, f64>,
        mut output: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<()> {
        let q = q.as_slice()?;
        let mut output = output.as_array_mut();
        if q.len() != self.dof() || output.shape() != [self.bodies(), 3] {
            return Err(PyValueError::new_err(
                "frame_positions array shape mismatch",
            ));
        }
        let mut state = RobotState::zeros(&self.program.model);
        state.q.as_mut_slice().copy_from_slice(q);
        let mut cache = ModelCache::new(&self.program.model);
        self.program
            .model
            .forward_kinematics(&state, &mut cache)
            .map_err(value_error)?;
        for body in &self.program.model.bodies {
            let translation = cache.world_from_body[body.id.0].translation.vector;
            for axis in 0..3 {
                output[[body.id.0, axis]] = translation[axis];
            }
        }
        Ok(())
    }

    fn center_of_mass(
        &self,
        q: PyReadonlyArray1<'_, f64>,
        mut output: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<()> {
        let q = q.as_slice()?;
        let output = output.as_slice_mut()?;
        if q.len() != self.dof() || output.len() != 3 {
            return Err(PyValueError::new_err("center_of_mass array shape mismatch"));
        }
        let mut state = RobotState::zeros(&self.program.model);
        state.q.as_mut_slice().copy_from_slice(q);
        let mut cache = ModelCache::new(&self.program.model);
        self.program
            .model
            .forward_kinematics(&state, &mut cache)
            .map_err(value_error)?;
        output.copy_from_slice(cache.center_of_mass_world.as_slice());
        Ok(())
    }

    /// Execute a complete fixed-shape trace in Rust.
    ///
    /// Input layouts:
    /// - `frame_ids[target]`
    /// - `target_positions[tick, target, xyz]`
    /// - `target_velocities[tick, target, xyz]`
    /// - `target_active[tick, target]`
    /// - `priorities[target]` where 0..4 map to the five fixed levels
    /// - `weights[target]`
    /// - `com_targets[tick, xyz]`, enabled by `com_active[tick]`
    /// - `posture[dof]`
    ///
    /// Outputs are preallocated by Python:
    /// - `q_out[tick, dof]`, `v_out[tick, dof]`
    /// - `tracked_positions_out[tick, target, xyz]`
    /// - `step_ns_out[tick]`, `status_out[tick]`
    #[allow(clippy::too_many_arguments)]
    fn run_trace(
        &mut self,
        start_time_ns: i64,
        frame_ids: PyReadonlyArray1<'_, i64>,
        target_positions: PyReadonlyArray3<'_, f64>,
        target_velocities: PyReadonlyArray3<'_, f64>,
        target_active: PyReadonlyArray2<'_, u8>,
        priorities: PyReadonlyArray1<'_, u8>,
        weights: PyReadonlyArray1<'_, f64>,
        com_targets: PyReadonlyArray2<'_, f64>,
        com_active: PyReadonlyArray1<'_, u8>,
        posture: PyReadonlyArray1<'_, f64>,
        mut q_out: PyReadwriteArray2<'_, f64>,
        mut v_out: PyReadwriteArray2<'_, f64>,
        mut tracked_positions_out: PyReadwriteArray3<'_, f64>,
        mut step_ns_out: PyReadwriteArray1<'_, u64>,
        mut status_out: PyReadwriteArray1<'_, u8>,
    ) -> PyResult<()> {
        let frame_ids = frame_ids.as_slice()?;
        let priorities = priorities.as_slice()?;
        let weights = weights.as_slice()?;
        let posture = posture.as_slice()?;
        let target_positions = target_positions.as_array();
        let target_velocities = target_velocities.as_array();
        let target_active = target_active.as_array();
        let com_targets = com_targets.as_array();
        let com_active = com_active.as_slice()?;
        let mut q_out = q_out.as_array_mut();
        let mut v_out = v_out.as_array_mut();
        let mut tracked_positions_out = tracked_positions_out.as_array_mut();
        let step_ns_out = step_ns_out.as_slice_mut()?;
        let status_out = status_out.as_slice_mut()?;

        let ticks = target_positions.shape()[0];
        let target_count = frame_ids.len();
        let dof = self.dof();
        let valid_shapes = target_positions.shape() == [ticks, target_count, 3]
            && target_velocities.shape() == [ticks, target_count, 3]
            && target_active.shape() == [ticks, target_count]
            && priorities.len() == target_count
            && weights.len() == target_count
            && com_targets.shape() == [ticks, 3]
            && com_active.len() == ticks
            && posture.len() == dof
            && q_out.shape() == [ticks, dof]
            && v_out.shape() == [ticks, dof]
            && tracked_positions_out.shape() == [ticks, target_count, 3]
            && step_ns_out.len() == ticks
            && status_out.len() == ticks;
        if !valid_shapes {
            return Err(PyValueError::new_err("run_trace array shape mismatch"));
        }
        for &frame in frame_ids {
            if frame < 0 || frame as usize >= self.program.model.bodies.len() {
                return Err(PyValueError::new_err("run_trace frame id out of range"));
            }
        }
        self.input.frame_targets.reserve(target_count);
        let input_posture = self
            .input
            .posture_target
            .as_mut()
            .expect("controller session always owns a posture buffer");
        input_posture.as_mut_slice().copy_from_slice(posture);

        for tick in 0..ticks {
            self.input.frame_targets.clear();
            for target in 0..target_count {
                if target_active[[tick, target]] == 0 {
                    continue;
                }
                self.input.frame_targets.push(FrameTarget {
                    stable_id: target as u32,
                    frame: bonesaw_core::FrameId(frame_ids[target] as usize),
                    point_in_frame: Vec3::zeros(),
                    target_position_world: Vec3::new(
                        target_positions[[tick, target, 0]],
                        target_positions[[tick, target, 1]],
                        target_positions[[tick, target, 2]],
                    ),
                    target_orientation_world: None,
                    feedforward_linear_velocity_world: Vec3::new(
                        target_velocities[[tick, target, 0]],
                        target_velocities[[tick, target, 1]],
                        target_velocities[[tick, target, 2]],
                    ),
                    priority: priority(priorities[target])?,
                    weight: weights[target],
                });
            }
            self.input.com_target_world = (com_active[tick] != 0).then(|| {
                Vec3::new(
                    com_targets[[tick, 0]],
                    com_targets[[tick, 1]],
                    com_targets[[tick, 2]],
                )
            });
            self.input.tick_time_ns =
                start_time_ns + tick as i64 * self.program.timing.control_horizon_ns;
            self.input.state.clone_from(&self.state_in.commanded_state);
            let started = Instant::now();
            self.controller
                .advance_into_reusable(
                    &self.input,
                    &self.state_in,
                    &mut self.state_out,
                    &mut self.output,
                    &mut self.scratch,
                )
                .map_err(value_error)?;
            step_ns_out[tick] = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let output = self
                .output
                .value
                .as_ref()
                .ok_or_else(|| PyValueError::new_err("controller produced no output"))?;
            status_out[tick] = match output.status {
                StepStatus::Ok => 0,
                StepStatus::Degraded => 1,
                StepStatus::Contingency => 2,
                StepStatus::Rejected => 3,
            };
            for coordinate in 0..dof {
                q_out[[tick, coordinate]] = output.next_state.q[coordinate];
                v_out[[tick, coordinate]] = output.next_state.v[coordinate];
            }
            for target in 0..target_count {
                let frame = &output.frames[frame_ids[target] as usize];
                for axis in 0..3 {
                    tracked_positions_out[[tick, target, axis]] = frame.translation[axis];
                }
            }
            std::mem::swap(&mut self.state_in, &mut self.state_out);
        }
        Ok(())
    }
}

#[pymethods]
impl KinematicWitnessSession {
    #[new]
    #[pyo3(signature = (urdf_path, target_capacity=4))]
    fn new(urdf_path: &str, target_capacity: usize) -> PyResult<Self> {
        if target_capacity == 0 || target_capacity > 32 {
            return Err(PyValueError::new_err("target_capacity must be in 1..=32"));
        }
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let state = RobotState::zeros(&program.model);
        let scratch = WholeBodyIkScratch::new(&program.model);
        let nominal_posture = DVector::zeros(program.model.dof);
        let initial_posture = DVector::zeros(program.model.dof);
        let jet_coordinate_regularization = DVector::from_element(program.model.dof, 1.0);
        let joint_acceleration = DVector::zeros(program.model.dof);
        Ok(Self {
            program,
            state,
            scratch,
            point_targets: Vec::with_capacity(target_capacity),
            jet_targets: Vec::with_capacity(target_capacity),
            target_rotations: Vec::with_capacity(target_capacity),
            nominal_posture,
            initial_posture,
            jet_coordinate_regularization,
            joint_acceleration,
        })
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn frame_names(&self) -> Vec<&str> {
        self.program
            .model
            .bodies
            .iter()
            .map(|body| body.name.as_str())
            .collect()
    }

    #[getter]
    fn joint_names(&self) -> Vec<&str> {
        self.program
            .model
            .joints
            .iter()
            .filter(|joint| joint.coordinate.is_some())
            .map(|joint| joint.name.as_str())
            .collect()
    }

    /// Generate a morphology-consistent joint position/velocity/acceleration
    /// witness. Every solve is in Rust; Python only supplies immutable arrays
    /// and caller-owned output buffers.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (
        dt_seconds,
        root_positions,
        center_of_mass_positions,
        frame_ids,
        target_positions,
        target_weights,
        initial_posture,
        q_out,
        v_out,
        acceleration_out,
        point_error_out,
        orientation_error_out,
        center_of_mass_error_out,
        iterations_out,
        converged_out,
        solve_ns_out,
        allocation_calls_out,
        allocated_bytes_out,
        maximum_iterations=32,
        minimum_iterations=0,
        damping=1e-6,
        posture_weight=1e-4,
        center_of_mass_weight=0.25,
        orientation_weight=100.0,
        maximum_step_rad=0.12,
        point_tolerance_m=1e-4,
        center_of_mass_tolerance_m=5e-3,
        orientation_tolerance_rad=1e-3
    ))]
    fn run_trace(
        &mut self,
        dt_seconds: f64,
        root_positions: PyReadonlyArray2<'_, f64>,
        center_of_mass_positions: PyReadonlyArray2<'_, f64>,
        frame_ids: PyReadonlyArray1<'_, i64>,
        target_positions: PyReadonlyArray3<'_, f64>,
        target_weights: PyReadonlyArray1<'_, f64>,
        initial_posture: PyReadonlyArray1<'_, f64>,
        mut q_out: PyReadwriteArray2<'_, f64>,
        mut v_out: PyReadwriteArray2<'_, f64>,
        mut acceleration_out: PyReadwriteArray2<'_, f64>,
        mut point_error_out: PyReadwriteArray1<'_, f64>,
        mut orientation_error_out: PyReadwriteArray1<'_, f64>,
        mut center_of_mass_error_out: PyReadwriteArray1<'_, f64>,
        mut iterations_out: PyReadwriteArray1<'_, u16>,
        mut converged_out: PyReadwriteArray1<'_, u8>,
        mut solve_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
        maximum_iterations: usize,
        minimum_iterations: usize,
        damping: f64,
        posture_weight: f64,
        center_of_mass_weight: f64,
        orientation_weight: f64,
        maximum_step_rad: f64,
        point_tolerance_m: f64,
        center_of_mass_tolerance_m: f64,
        orientation_tolerance_rad: f64,
    ) -> PyResult<()> {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return Err(PyValueError::new_err("dt_seconds must be positive"));
        }
        let root_positions = root_positions.as_array();
        let center_of_mass_positions = center_of_mass_positions.as_array();
        let frame_ids = frame_ids.as_slice()?;
        let target_positions = target_positions.as_array();
        let target_weights = target_weights.as_slice()?;
        let initial_posture = initial_posture.as_slice()?;
        let mut q_out = q_out.as_array_mut();
        let mut v_out = v_out.as_array_mut();
        let mut acceleration_out = acceleration_out.as_array_mut();
        let point_error_out = point_error_out.as_slice_mut()?;
        let orientation_error_out = orientation_error_out.as_slice_mut()?;
        let center_of_mass_error_out = center_of_mass_error_out.as_slice_mut()?;
        let iterations_out = iterations_out.as_slice_mut()?;
        let converged_out = converged_out.as_slice_mut()?;
        let solve_ns_out = solve_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let ticks = root_positions.shape()[0];
        let target_count = frame_ids.len();
        let dof = self.dof();
        if ticks < 3
            || target_count > self.point_targets.capacity()
            || root_positions.shape() != [ticks, 3]
            || center_of_mass_positions.shape() != [ticks, 3]
            || target_positions.shape() != [ticks, target_count, 3]
            || target_weights.len() != target_count
            || initial_posture.len() != dof
            || q_out.shape() != [ticks, dof]
            || v_out.shape() != [ticks, dof]
            || acceleration_out.shape() != [ticks, dof]
            || point_error_out.len() != ticks
            || orientation_error_out.len() != ticks
            || center_of_mass_error_out.len() != ticks
            || iterations_out.len() != ticks
            || converged_out.len() != ticks
            || solve_ns_out.len() != ticks
            || allocation_calls_out.len() != ticks
            || allocated_bytes_out.len() != ticks
        {
            return Err(PyValueError::new_err(
                "kinematic witness array shape or capacity mismatch",
            ));
        }
        if frame_ids
            .iter()
            .any(|&frame| frame < 0 || frame as usize >= self.program.model.bodies.len())
            || target_weights
                .iter()
                .any(|weight| !weight.is_finite() || *weight < 0.0)
            || initial_posture.iter().any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "kinematic witness contains an invalid frame, weight, or posture",
            ));
        }
        self.state.q.as_mut_slice().copy_from_slice(initial_posture);
        self.state.v.fill(0.0);
        self.initial_posture
            .as_mut_slice()
            .copy_from_slice(initial_posture);
        let options = WholeBodyIkOptions {
            maximum_iterations,
            minimum_iterations,
            damping,
            posture_weight,
            center_of_mass_weight,
            orientation_weight,
            maximum_step_rad,
            point_tolerance_m,
            center_of_mass_tolerance_m,
            orientation_tolerance_rad,
        };
        for axis in 0..3 {
            self.state.control_world_from_root.translation.vector[axis] = root_positions[[0, axis]];
        }
        self.program
            .model
            .forward_kinematics(&self.state, &mut self.scratch.model)
            .map_err(value_error)?;
        self.target_rotations.clear();
        for &frame in frame_ids {
            self.target_rotations
                .push(self.scratch.model.world_from_body[frame as usize].rotation);
        }
        for tick in 0..ticks {
            // Keep most regularization local to the warm start, while a small
            // standing-branch guide prevents a fully extended leg from
            // becoming a singular local minimum immediately after liftoff.
            for coordinate in 0..dof {
                self.nominal_posture[coordinate] =
                    0.95 * self.state.q[coordinate] + 0.05 * self.initial_posture[coordinate];
            }
            for axis in 0..3 {
                self.state.control_world_from_root.translation.vector[axis] =
                    root_positions[[tick, axis]];
            }
            self.point_targets.clear();
            for target in 0..target_count {
                self.point_targets.push(WholeBodyPointIkTarget {
                    frame: bonesaw_core::FrameId(frame_ids[target] as usize),
                    point_in_frame: Vec3::zeros(),
                    target_world: Vec3::new(
                        target_positions[[tick, target, 0]],
                        target_positions[[tick, target, 1]],
                        target_positions[[tick, target, 2]],
                    ),
                    weight: target_weights[target],
                    target_orientation_world: self.target_rotations[target],
                    orientation_weight: target_weights[target],
                });
            }
            let center_of_mass_target = Vec3::new(
                center_of_mass_positions[[tick, 0]],
                center_of_mass_positions[[tick, 1]],
                center_of_mass_positions[[tick, 2]],
            );
            let (allocation_calls_before, allocated_bytes_before) = allocation_snapshot();
            let started = Instant::now();
            let report = solve_whole_body_ik_into(
                &self.program.model,
                &mut self.state,
                &self.point_targets,
                Some(center_of_mass_target),
                &self.nominal_posture,
                options,
                &mut self.scratch,
            )
            .map_err(value_error)?;
            solve_ns_out[tick] = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let (allocation_calls_after, allocated_bytes_after) = allocation_snapshot();
            allocation_calls_out[tick] =
                allocation_calls_after.saturating_sub(allocation_calls_before);
            allocated_bytes_out[tick] =
                allocated_bytes_after.saturating_sub(allocated_bytes_before);
            point_error_out[tick] = report.maximum_point_error_m;
            orientation_error_out[tick] = report.maximum_orientation_error_rad;
            center_of_mass_error_out[tick] = report.center_of_mass_error_m;
            iterations_out[tick] = report.iterations.min(u16::MAX as usize) as u16;
            converged_out[tick] = u8::from(report.converged);
            for coordinate in 0..dof {
                q_out[[tick, coordinate]] = self.state.q[coordinate];
            }
        }
        let inverse_dt = 1.0 / dt_seconds;
        let inverse_two_dt = 0.5 * inverse_dt;
        let inverse_dt_squared = inverse_dt * inverse_dt;
        for coordinate in 0..dof {
            v_out[[0, coordinate]] = (q_out[[1, coordinate]] - q_out[[0, coordinate]]) * inverse_dt;
            acceleration_out[[0, coordinate]] =
                (q_out[[2, coordinate]] - 2.0 * q_out[[1, coordinate]] + q_out[[0, coordinate]])
                    * inverse_dt_squared;
            for tick in 1..ticks - 1 {
                v_out[[tick, coordinate]] = (q_out[[tick + 1, coordinate]]
                    - q_out[[tick - 1, coordinate]])
                    * inverse_two_dt;
                acceleration_out[[tick, coordinate]] = (q_out[[tick + 1, coordinate]]
                    - 2.0 * q_out[[tick, coordinate]]
                    + q_out[[tick - 1, coordinate]])
                    * inverse_dt_squared;
            }
            v_out[[ticks - 1, coordinate]] =
                (q_out[[ticks - 1, coordinate]] - q_out[[ticks - 2, coordinate]]) * inverse_dt;
            acceleration_out[[ticks - 1, coordinate]] = (q_out[[ticks - 1, coordinate]]
                - 2.0 * q_out[[ticks - 2, coordinate]]
                + q_out[[ticks - 3, coordinate]])
                * inverse_dt_squared;
        }
        Ok(())
    }

    /// Solve analytic velocity and acceleration jets for a position witness.
    #[allow(clippy::too_many_arguments)]
    fn run_jet_trace(
        &mut self,
        root_positions: PyReadonlyArray2<'_, f64>,
        root_velocities: PyReadonlyArray2<'_, f64>,
        root_accelerations: PyReadonlyArray2<'_, f64>,
        center_of_mass_velocities: PyReadonlyArray2<'_, f64>,
        center_of_mass_accelerations: PyReadonlyArray2<'_, f64>,
        frame_ids: PyReadonlyArray1<'_, i64>,
        target_velocities: PyReadonlyArray3<'_, f64>,
        target_accelerations: PyReadonlyArray3<'_, f64>,
        target_weights: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray2<'_, f64>,
        mut v_out: PyReadwriteArray2<'_, f64>,
        mut acceleration_out: PyReadwriteArray2<'_, f64>,
        mut point_velocity_residual_out: PyReadwriteArray1<'_, f64>,
        mut point_acceleration_residual_out: PyReadwriteArray1<'_, f64>,
        mut angular_velocity_residual_out: PyReadwriteArray1<'_, f64>,
        mut angular_acceleration_residual_out: PyReadwriteArray1<'_, f64>,
        mut center_of_mass_velocity_residual_out: PyReadwriteArray1<'_, f64>,
        mut center_of_mass_acceleration_residual_out: PyReadwriteArray1<'_, f64>,
        mut solve_ns_out: PyReadwriteArray1<'_, u64>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
        velocity_damping: f64,
        acceleration_damping: f64,
        center_of_mass_weight: f64,
        off_chain_regularization: f64,
        strict_effector_targets: bool,
    ) -> PyResult<()> {
        let root_positions = root_positions.as_array();
        let root_velocities = root_velocities.as_array();
        let root_accelerations = root_accelerations.as_array();
        let center_of_mass_velocities = center_of_mass_velocities.as_array();
        let center_of_mass_accelerations = center_of_mass_accelerations.as_array();
        let frame_ids = frame_ids.as_slice()?;
        let target_velocities = target_velocities.as_array();
        let target_accelerations = target_accelerations.as_array();
        let target_weights = target_weights.as_slice()?;
        let q = q.as_array();
        let mut v_out = v_out.as_array_mut();
        let mut acceleration_out = acceleration_out.as_array_mut();
        let point_velocity_residual_out = point_velocity_residual_out.as_slice_mut()?;
        let point_acceleration_residual_out = point_acceleration_residual_out.as_slice_mut()?;
        let angular_velocity_residual_out = angular_velocity_residual_out.as_slice_mut()?;
        let angular_acceleration_residual_out = angular_acceleration_residual_out.as_slice_mut()?;
        let center_of_mass_velocity_residual_out =
            center_of_mass_velocity_residual_out.as_slice_mut()?;
        let center_of_mass_acceleration_residual_out =
            center_of_mass_acceleration_residual_out.as_slice_mut()?;
        let solve_ns_out = solve_ns_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let ticks = root_positions.shape()[0];
        let target_count = frame_ids.len();
        let dof = self.dof();
        if target_count > self.jet_targets.capacity()
            || root_positions.shape() != [ticks, 3]
            || root_velocities.shape() != [ticks, 3]
            || root_accelerations.shape() != [ticks, 3]
            || center_of_mass_velocities.shape() != [ticks, 3]
            || center_of_mass_accelerations.shape() != [ticks, 3]
            || target_velocities.shape() != [ticks, target_count, 3]
            || target_accelerations.shape() != [ticks, target_count, 3]
            || target_weights.len() != target_count
            || q.shape() != [ticks, dof]
            || v_out.shape() != [ticks, dof]
            || acceleration_out.shape() != [ticks, dof]
            || point_velocity_residual_out.len() != ticks
            || point_acceleration_residual_out.len() != ticks
            || angular_velocity_residual_out.len() != ticks
            || angular_acceleration_residual_out.len() != ticks
            || center_of_mass_velocity_residual_out.len() != ticks
            || center_of_mass_acceleration_residual_out.len() != ticks
            || solve_ns_out.len() != ticks
            || allocation_calls_out.len() != ticks
            || allocated_bytes_out.len() != ticks
        {
            return Err(PyValueError::new_err(
                "kinematic jet array shape or capacity mismatch",
            ));
        }
        if frame_ids
            .iter()
            .any(|&frame| frame < 0 || frame as usize >= self.program.model.bodies.len())
            || target_weights
                .iter()
                .any(|weight| !weight.is_finite() || *weight < 0.0)
            || !off_chain_regularization.is_finite()
            || off_chain_regularization < 1.0
        {
            return Err(PyValueError::new_err(
                "kinematic jet frame or weight is invalid",
            ));
        }
        self.jet_coordinate_regularization
            .fill(off_chain_regularization);
        for &frame in frame_ids {
            let mut body = bonesaw_core::BodyId(frame as usize);
            while let Some(joint_id) = self.program.model.bodies[body.0].parent_joint {
                let joint = &self.program.model.joints[joint_id.0];
                if let Some(coordinate) = joint.coordinate {
                    self.jet_coordinate_regularization[coordinate] = 1.0;
                }
                body = joint.parent;
            }
        }
        let options = WholeBodyJetOptions {
            velocity_damping,
            acceleration_damping,
            center_of_mass_weight,
            strict_effector_targets,
        };
        for tick in 0..ticks {
            self.state.q.as_mut_slice().copy_from_slice(
                q.row(tick)
                    .as_slice()
                    .ok_or_else(|| PyValueError::new_err("q rows must be contiguous"))?,
            );
            self.state.control_world_from_root.rotation = UnitQuaternion::identity();
            let mut root_twist = Motion6::default();
            for axis in 0..3 {
                self.state.control_world_from_root.translation.vector[axis] =
                    root_positions[[tick, axis]];
                root_twist.0[3 + axis] = root_velocities[[tick, axis]];
            }
            self.jet_targets.clear();
            for target in 0..target_count {
                self.jet_targets.push(WholeBodyPointJetTarget {
                    frame: bonesaw_core::FrameId(frame_ids[target] as usize),
                    point_in_frame: Vec3::zeros(),
                    target_velocity_world: Vec3::new(
                        target_velocities[[tick, target, 0]],
                        target_velocities[[tick, target, 1]],
                        target_velocities[[tick, target, 2]],
                    ),
                    target_acceleration_world: Vec3::new(
                        target_accelerations[[tick, target, 0]],
                        target_accelerations[[tick, target, 1]],
                        target_accelerations[[tick, target, 2]],
                    ),
                    weight: target_weights[target],
                    target_angular_velocity_world: Vec3::zeros(),
                    target_angular_acceleration_world: Vec3::zeros(),
                    angular_weight: target_weights[target],
                });
            }
            let (allocation_calls_before, allocated_bytes_before) = allocation_snapshot();
            let started = Instant::now();
            let report = solve_whole_body_kinematic_jets_into(
                &self.program.model,
                &mut self.state,
                root_twist,
                Vec3::zeros(),
                Vec3::new(
                    root_accelerations[[tick, 0]],
                    root_accelerations[[tick, 1]],
                    root_accelerations[[tick, 2]],
                ),
                &self.jet_targets,
                Some(Vec3::new(
                    center_of_mass_velocities[[tick, 0]],
                    center_of_mass_velocities[[tick, 1]],
                    center_of_mass_velocities[[tick, 2]],
                )),
                Some(Vec3::new(
                    center_of_mass_accelerations[[tick, 0]],
                    center_of_mass_accelerations[[tick, 1]],
                    center_of_mass_accelerations[[tick, 2]],
                )),
                Some(&self.jet_coordinate_regularization),
                &mut self.joint_acceleration,
                options,
                &mut self.scratch,
            )
            .map_err(value_error)?;
            solve_ns_out[tick] = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let (allocation_calls_after, allocated_bytes_after) = allocation_snapshot();
            allocation_calls_out[tick] =
                allocation_calls_after.saturating_sub(allocation_calls_before);
            allocated_bytes_out[tick] =
                allocated_bytes_after.saturating_sub(allocated_bytes_before);
            point_velocity_residual_out[tick] = report.maximum_point_velocity_residual_mps;
            point_acceleration_residual_out[tick] = report.maximum_point_acceleration_residual_mps2;
            angular_velocity_residual_out[tick] = report.maximum_angular_velocity_residual_rad_s;
            angular_acceleration_residual_out[tick] =
                report.maximum_angular_acceleration_residual_rad_s2;
            center_of_mass_velocity_residual_out[tick] =
                report.center_of_mass_velocity_residual_mps;
            center_of_mass_acceleration_residual_out[tick] =
                report.center_of_mass_acceleration_residual_mps2;
            for coordinate in 0..dof {
                v_out[[tick, coordinate]] = self.state.v[coordinate];
                acceleration_out[[tick, coordinate]] = self.joint_acceleration[coordinate];
            }
        }
        Ok(())
    }
}

#[pymethods]
impl FloatingWbcSession {
    #[new]
    #[pyo3(signature = (
        urdf_path,
        maximum_contacts=2,
        friction_coefficient=1.0,
        maximum_acceleration=200.0,
        maximum_torque=2000.0,
        maximum_normal_force_multiple=3.0,
        maximum_feasibility_iterations=64,
        maximum_feasibility_projection_sweeps=None,
        feasibility_projection_continuation_violation_threshold=None,
        repair_feasibility_equalities_before_inequalities=false,
        use_feasibility_row_spans=false,
        reuse_identical_hard_feasibility_seed=false,
        joint_limit_braking=false,
        root_frequency_hz=2.0,
        root_angular_task_weight=1.0,
        root_height_task_weight=10.0,
        root_horizontal_task_weight=1.0,
        root_horizontal_task_priority=2,
        point_frequency_hz=4.0,
        joint_posture_weight=0.25,
        joint_posture_priority=3,
        center_of_mass_task_weight=0.0,
        center_of_mass_task_priority=1,
        center_of_mass_frequency_hz=2.0,
        dcm_balance_enabled=false,
        dcm_feedback_gain_per_second=3.5,
        dcm_support_margin_m=0.01,
        dcm_maximum_horizontal_acceleration_mps2=25.0,
        protected_joint_posture_weight=0.0,
        protected_joint_posture_priority=2,
        joint_velocity_envelope_weight=0.0,
        joint_velocity_envelope_priority=1,
        joint_velocity_envelope_activation_fraction=0.75,
        joint_velocity_envelope_frequency_hz=2.0,
        joint_velocity_envelope_multi_support_only=false,
        joint_velocity_envelope_phase_transition_ticks=0,
        balance_feedback_authority_enabled=false,
        capture_landing_retarget_enabled=false,
        capture_landing_activation_margin_m=0.0,
        capture_landing_full_scale_margin_m=-0.10,
        capture_landing_maximum_authored_offset_m=0.12,
        capture_landing_maximum_root_reach_m=0.90,
        capture_landing_maximum_anchor_speed_mps=0.50,
        capture_landing_freeze_ticks=40,
        touchdown_phase_retiming_enabled=false,
        touchdown_phase_minimum_rate=0.0,
        touchdown_phase_guard_time_seconds=0.02,
        touchdown_phase_release_ticks=100,
        touchdown_phase_engagement_ticks=0,
        balance_phase_retiming_enabled=false,
        balance_phase_hold_margin_m=-0.02,
        balance_phase_full_rate_margin_m=0.0,
        centroidal_angular_momentum_weight=0.0,
        centroidal_angular_momentum_priority=2,
        centroidal_angular_momentum_frequency_hz=1.0,
        precontact_ticks=0,
        precontact_maximum_acceleration=25.0,
        material_touchdown_task=false,
        contact_patch_center_x=0.0,
        contact_patch_half_length=0.0,
        contact_patch_half_width=0.0,
        contact_patch_z=0.0,
        minimum_contact_cop_margin_m=0.0
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        urdf_path: &str,
        maximum_contacts: usize,
        friction_coefficient: f64,
        maximum_acceleration: f64,
        maximum_torque: f64,
        maximum_normal_force_multiple: f64,
        maximum_feasibility_iterations: usize,
        maximum_feasibility_projection_sweeps: Option<usize>,
        feasibility_projection_continuation_violation_threshold: Option<f64>,
        repair_feasibility_equalities_before_inequalities: bool,
        use_feasibility_row_spans: bool,
        reuse_identical_hard_feasibility_seed: bool,
        joint_limit_braking: bool,
        root_frequency_hz: f64,
        root_angular_task_weight: f64,
        root_height_task_weight: f64,
        root_horizontal_task_weight: f64,
        root_horizontal_task_priority: u8,
        point_frequency_hz: f64,
        joint_posture_weight: f64,
        joint_posture_priority: u8,
        center_of_mass_task_weight: f64,
        center_of_mass_task_priority: u8,
        center_of_mass_frequency_hz: f64,
        dcm_balance_enabled: bool,
        dcm_feedback_gain_per_second: f64,
        dcm_support_margin_m: f64,
        dcm_maximum_horizontal_acceleration_mps2: f64,
        protected_joint_posture_weight: f64,
        protected_joint_posture_priority: u8,
        joint_velocity_envelope_weight: f64,
        joint_velocity_envelope_priority: u8,
        joint_velocity_envelope_activation_fraction: f64,
        joint_velocity_envelope_frequency_hz: f64,
        joint_velocity_envelope_multi_support_only: bool,
        joint_velocity_envelope_phase_transition_ticks: usize,
        balance_feedback_authority_enabled: bool,
        capture_landing_retarget_enabled: bool,
        capture_landing_activation_margin_m: f64,
        capture_landing_full_scale_margin_m: f64,
        capture_landing_maximum_authored_offset_m: f64,
        capture_landing_maximum_root_reach_m: f64,
        capture_landing_maximum_anchor_speed_mps: f64,
        capture_landing_freeze_ticks: usize,
        touchdown_phase_retiming_enabled: bool,
        touchdown_phase_minimum_rate: f64,
        touchdown_phase_guard_time_seconds: f64,
        touchdown_phase_release_ticks: usize,
        touchdown_phase_engagement_ticks: usize,
        balance_phase_retiming_enabled: bool,
        balance_phase_hold_margin_m: f64,
        balance_phase_full_rate_margin_m: f64,
        centroidal_angular_momentum_weight: f64,
        centroidal_angular_momentum_priority: u8,
        centroidal_angular_momentum_frequency_hz: f64,
        precontact_ticks: usize,
        precontact_maximum_acceleration: f64,
        material_touchdown_task: bool,
        contact_patch_center_x: f64,
        contact_patch_half_length: f64,
        contact_patch_half_width: f64,
        contact_patch_z: f64,
        minimum_contact_cop_margin_m: f64,
    ) -> PyResult<Self> {
        if maximum_contacts == 0 || maximum_contacts > 16 {
            return Err(PyValueError::new_err("maximum_contacts must be in 1..=16"));
        }
        if !(1..=64).contains(&maximum_feasibility_iterations) {
            return Err(PyValueError::new_err(
                "maximum_feasibility_iterations must be in 1..=64",
            ));
        }
        if maximum_feasibility_projection_sweeps.is_some_and(|sweeps| sweeps == 0) {
            return Err(PyValueError::new_err(
                "maximum_feasibility_projection_sweeps must be positive when provided",
            ));
        }
        if feasibility_projection_continuation_violation_threshold
            .is_some_and(|threshold| !threshold.is_finite() || threshold < 0.0)
        {
            return Err(PyValueError::new_err(
                "feasibility projection continuation threshold must be finite and nonnegative",
            ));
        }
        if !friction_coefficient.is_finite() || friction_coefficient < 0.0 {
            return Err(PyValueError::new_err(
                "friction_coefficient must be finite and nonnegative",
            ));
        }
        if !maximum_acceleration.is_finite()
            || maximum_acceleration <= 0.0
            || !maximum_torque.is_finite()
            || maximum_torque <= 0.0
            || !maximum_normal_force_multiple.is_finite()
            || maximum_normal_force_multiple <= 0.0
            || !root_frequency_hz.is_finite()
            || root_frequency_hz <= 0.0
            || !root_angular_task_weight.is_finite()
            || root_angular_task_weight < 0.0
            || !root_height_task_weight.is_finite()
            || root_height_task_weight < 0.0
            || !root_horizontal_task_weight.is_finite()
            || root_horizontal_task_weight < 0.0
            || !point_frequency_hz.is_finite()
            || point_frequency_hz <= 0.0
            || !joint_posture_weight.is_finite()
            || joint_posture_weight < 0.0
            || !center_of_mass_task_weight.is_finite()
            || center_of_mass_task_weight < 0.0
            || !center_of_mass_frequency_hz.is_finite()
            || center_of_mass_frequency_hz <= 0.0
            || !dcm_feedback_gain_per_second.is_finite()
            || dcm_feedback_gain_per_second <= 0.0
            || !dcm_support_margin_m.is_finite()
            || dcm_support_margin_m < 0.0
            || !dcm_maximum_horizontal_acceleration_mps2.is_finite()
            || dcm_maximum_horizontal_acceleration_mps2 <= 0.0
            || !protected_joint_posture_weight.is_finite()
            || protected_joint_posture_weight < 0.0
            || !joint_velocity_envelope_weight.is_finite()
            || joint_velocity_envelope_weight < 0.0
            || !joint_velocity_envelope_activation_fraction.is_finite()
            || !(0.0..1.0).contains(&joint_velocity_envelope_activation_fraction)
            || !joint_velocity_envelope_frequency_hz.is_finite()
            || joint_velocity_envelope_frequency_hz <= 0.0
            || !capture_landing_activation_margin_m.is_finite()
            || !capture_landing_full_scale_margin_m.is_finite()
            || capture_landing_full_scale_margin_m >= capture_landing_activation_margin_m
            || !capture_landing_maximum_authored_offset_m.is_finite()
            || capture_landing_maximum_authored_offset_m < 0.0
            || !capture_landing_maximum_root_reach_m.is_finite()
            || capture_landing_maximum_root_reach_m <= 0.0
            || !capture_landing_maximum_anchor_speed_mps.is_finite()
            || capture_landing_maximum_anchor_speed_mps <= 0.0
            || !touchdown_phase_minimum_rate.is_finite()
            || !(0.0..=1.0).contains(&touchdown_phase_minimum_rate)
            || !touchdown_phase_guard_time_seconds.is_finite()
            || touchdown_phase_guard_time_seconds < 0.0
            || !balance_phase_hold_margin_m.is_finite()
            || !balance_phase_full_rate_margin_m.is_finite()
            || balance_phase_hold_margin_m >= balance_phase_full_rate_margin_m
            || !centroidal_angular_momentum_weight.is_finite()
            || centroidal_angular_momentum_weight < 0.0
            || !centroidal_angular_momentum_frequency_hz.is_finite()
            || centroidal_angular_momentum_frequency_hz <= 0.0
            || !precontact_maximum_acceleration.is_finite()
            || precontact_maximum_acceleration <= 0.0
        {
            return Err(PyValueError::new_err(
                "limits must be finite and positive, and CoM weight finite and nonnegative",
            ));
        }
        if !contact_patch_center_x.is_finite()
            || !contact_patch_half_length.is_finite()
            || !contact_patch_half_width.is_finite()
            || !contact_patch_z.is_finite()
            || !minimum_contact_cop_margin_m.is_finite()
            || contact_patch_half_length < 0.0
            || contact_patch_half_width < 0.0
            || minimum_contact_cop_margin_m < 0.0
        {
            return Err(PyValueError::new_err(
                "contact-patch dimensions must be finite and half extents nonnegative",
            ));
        }
        let center_of_mass_task_priority = priority(center_of_mass_task_priority)?;
        let root_horizontal_task_priority = priority(root_horizontal_task_priority)?;
        let joint_posture_priority = priority(joint_posture_priority)?;
        let protected_joint_posture_priority = priority(protected_joint_posture_priority)?;
        let joint_velocity_envelope_priority = priority(joint_velocity_envelope_priority)?;
        let centroidal_angular_momentum_priority = priority(centroidal_angular_momentum_priority)?;
        let contact_points_per_target =
            if contact_patch_half_length > 0.0 || contact_patch_half_width > 0.0 {
                4
            } else {
                1
            };
        if minimum_contact_cop_margin_m > 0.0
            && (contact_points_per_target != 4
                || minimum_contact_cop_margin_m >= contact_patch_half_length
                || minimum_contact_cop_margin_m >= contact_patch_half_width)
        {
            return Err(PyValueError::new_err(
                "minimum_contact_cop_margin_m requires a four-point patch and must be smaller than both half extents",
            ));
        }
        if dcm_balance_enabled
            && (contact_points_per_target != 4 || center_of_mass_task_weight <= 0.0)
        {
            return Err(PyValueError::new_err(
                "DCM balance requires a finite four-point support patch and a positive CoM task weight",
            ));
        }
        if balance_feedback_authority_enabled && !dcm_balance_enabled {
            return Err(PyValueError::new_err(
                "balance-feedback authority requires DCM balance telemetry",
            ));
        }
        if capture_landing_retarget_enabled && !dcm_balance_enabled {
            return Err(PyValueError::new_err(
                "capture landing retargeting requires DCM balance telemetry",
            ));
        }
        if touchdown_phase_retiming_enabled && precontact_ticks == 0 {
            return Err(PyValueError::new_err(
                "touchdown phase retiming requires a positive precontact horizon",
            ));
        }
        if balance_phase_retiming_enabled && !dcm_balance_enabled {
            return Err(PyValueError::new_err(
                "balance phase retiming requires DCM balance telemetry",
            ));
        }
        if protected_joint_posture_weight > 0.0 && joint_velocity_envelope_weight > 0.0 {
            return Err(PyValueError::new_err(
                "protected posture and joint-velocity envelope share one fixed task slot; enable at most one",
            ));
        }
        let contact_patch_points = if contact_points_per_target == 4 {
            [
                Vec3::new(
                    contact_patch_center_x - contact_patch_half_length,
                    -contact_patch_half_width,
                    contact_patch_z,
                ),
                Vec3::new(
                    contact_patch_center_x - contact_patch_half_length,
                    contact_patch_half_width,
                    contact_patch_z,
                ),
                Vec3::new(
                    contact_patch_center_x + contact_patch_half_length,
                    -contact_patch_half_width,
                    contact_patch_z,
                ),
                Vec3::new(
                    contact_patch_center_x + contact_patch_half_length,
                    contact_patch_half_width,
                    contact_patch_z,
                ),
            ]
        } else {
            [
                Vec3::new(contact_patch_center_x, 0.0, contact_patch_z),
                Vec3::zeros(),
                Vec3::zeros(),
                Vec3::zeros(),
            ]
        };
        let program = MotionProgram::compile_urdf_file(urdf_path, TimingSpec::default(), 1)
            .map_err(value_error)?;
        let generalized_effort_limits = program
            .actuation
            .independent_generalized_effort_limits()
            .ok_or_else(|| {
            PyValueError::new_err(
                "floating WBC currently requires an independent diagonal actuation map",
            )
        })?;
        let model = &program.model;
        let generalized_dof = model.dof + 6;
        let mut joint_velocity_limits = vec![f64::INFINITY; model.dof];
        for joint in &model.joints {
            if let Some(coordinate) = joint.coordinate {
                joint_velocity_limits[coordinate] = joint.limit.velocity.abs().min(8.0);
            }
        }
        let controller = FloatingDynamicWbc::new(
            model.clone(),
            DynamicWbcConfig {
                maximum_feasibility_iterations,
                maximum_feasibility_projection_sweeps,
                feasibility_projection_continuation_violation_threshold,
                repair_feasibility_equalities_before_inequalities,
                use_feasibility_row_spans,
                reuse_identical_hard_feasibility_seed,
                ..DynamicWbcConfig::default()
            },
        )
        .map_err(value_error)?;
        let state = FloatingRobotState::zeros(model);
        let scratch = FloatingDynamicWbcScratch::new(model, maximum_contacts);
        let realization_scratch =
            FloatingDynamicWbcScratch::new_fixed_effort(model, maximum_contacts);
        let output = FloatingDynamicWbcOutput::workspace(
            model.dof,
            maximum_contacts,
            generalized_dof + maximum_contacts * 8 + model.dof,
        );
        let supported_weight = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
        let mut torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -maximum_torque),
            upper: DVector::from_element(model.dof, maximum_torque),
        };
        for (coordinate, effort) in generalized_effort_limits.into_iter().enumerate() {
            let limit = effort.min(maximum_torque);
            torque_bounds.lower[coordinate] = -limit;
            torque_bounds.upper[coordinate] = limit;
        }
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -maximum_acceleration),
            upper: DVector::from_element(generalized_dof, maximum_acceleration),
        };
        let nominal_acceleration_bounds = acceleration_bounds.clone();
        let nominal_torque_bounds = torque_bounds.clone();
        let actuator_effort_bounds = torque_bounds.clone();
        let nominal_actuator_effort_bounds = actuator_effort_bounds.clone();
        Ok(Self {
            controller,
            state,
            scratch,
            realization_scratch,
            output,
            acceleration_bounds,
            nominal_acceleration_bounds,
            torque_bounds,
            nominal_torque_bounds,
            actuator_effort_bounds,
            nominal_actuator_effort_bounds,
            mapped_actuator_effort: DVector::zeros(model.dof),
            coupled_actuation_enabled: false,
            maximum_generalized_torque: maximum_torque,
            desired_acceleration: DVector::zeros(generalized_dof),
            protected_joint_coordinates: Vec::with_capacity(model.dof),
            protected_joint_accelerations: vec![0.0; model.dof],
            velocity_envelope_coordinates: Vec::with_capacity(model.dof),
            velocity_envelope_accelerations: Vec::with_capacity(model.dof),
            joint_velocity_limits,
            tracking_cache: ModelCache::new(model),
            centroidal_dynamics: DynamicsCache::new(model),
            contact_transition_response_scratch: ContactTransitionResponseScratch::new(model),
            centroidal_map: DMatrix::zeros(6, generalized_dof),
            tracking_jacobian: DMatrix::zeros(3, generalized_dof),
            generalized_velocity: DVector::zeros(generalized_dof),
            previous_center_of_mass_world: Vec3::zeros(),
            point_tasks: Vec::with_capacity(FLOATING_POINT_TASK_CAPACITY),
            angular_tasks: Vec::with_capacity(maximum_contacts),
            contacts: Vec::with_capacity(maximum_contacts),
            support_patches: Vec::with_capacity(FLOATING_POINT_TASK_CAPACITY),
            support_transitions: [SupportTransitionState::default(); FLOATING_POINT_TASK_CAPACITY],
            contact_release_suppressed: [false; FLOATING_POINT_TASK_CAPACITY],
            no_contact_safe_mode: false,
            support_transition_config: SupportTransitionConfig::default(),
            precontact_authored_anchor_world: [Vec3::zeros(); FLOATING_POINT_TASK_CAPACITY],
            precontact_anchor_world: [Vec3::zeros(); FLOATING_POINT_TASK_CAPACITY],
            precontact_future_root_world: [Vec3::zeros(); FLOATING_POINT_TASK_CAPACITY],
            precontact_rotation_world: [UnitQuaternion::identity(); FLOATING_POINT_TASK_CAPACITY],
            precontact_planned: [false; FLOATING_POINT_TASK_CAPACITY],
            contact_anchor_world: [Vec3::zeros(); FLOATING_POINT_TASK_CAPACITY],
            contact_patch_anchor_world: [[Vec3::zeros(); 4]; FLOATING_POINT_TASK_CAPACITY],
            contact_patch_points,
            contact_points_per_target,
            minimum_contact_cop_margin_m,
            maximum_contacts,
            precontact_ticks,
            precontact_maximum_acceleration,
            material_touchdown_task,
            contact_friction_coefficient: friction_coefficient,
            maximum_normal_force_multiple,
            maximum_acceleration,
            joint_limit_braking,
            root_omega: std::f64::consts::TAU * root_frequency_hz,
            root_angular_task_weight,
            root_height_task_weight,
            root_horizontal_task_weight,
            root_horizontal_task_priority,
            point_omega: std::f64::consts::TAU * point_frequency_hz,
            joint_posture_weight,
            joint_posture_priority,
            center_of_mass_task_weight,
            center_of_mass_task_priority,
            center_of_mass_omega: std::f64::consts::TAU * center_of_mass_frequency_hz,
            dcm_balance_enabled,
            dcm_balance_config: DcmBalanceConfig {
                feedback_gain_per_second: dcm_feedback_gain_per_second,
                support_margin_m: dcm_support_margin_m,
                maximum_horizontal_acceleration_mps2: dcm_maximum_horizontal_acceleration_mps2,
                ..DcmBalanceConfig::default()
            },
            protected_joint_posture_weight,
            protected_joint_posture_priority,
            joint_velocity_envelope_weight,
            joint_velocity_envelope_priority,
            joint_velocity_envelope_activation_fraction,
            joint_velocity_envelope_omega: std::f64::consts::TAU
                * joint_velocity_envelope_frequency_hz,
            contact_phase_authority_config: if joint_velocity_envelope_multi_support_only {
                ContactPhaseAuthorityConfig {
                    unsupported_joint_velocity_scale: 0.0,
                    single_support_joint_velocity_scale: if balance_feedback_authority_enabled {
                        1.0
                    } else {
                        0.0
                    },
                    precontact_joint_velocity_scale: 0.0,
                    multi_support_joint_velocity_scale: 1.0,
                }
            } else {
                ContactPhaseAuthorityConfig::default()
            },
            contact_phase_authority_scale: if joint_velocity_envelope_multi_support_only {
                0.0
            } else {
                1.0
            },
            contact_phase_authority_maximum_delta_per_tick:
                if joint_velocity_envelope_phase_transition_ticks == 0 {
                    f64::INFINITY
                } else {
                    1.0 / joint_velocity_envelope_phase_transition_ticks as f64
                },
            balance_feedback_authority_enabled,
            balance_feedback_authority_config: BalanceFeedbackAuthorityConfig::default(),
            capture_landing_retarget_enabled,
            capture_landing_retarget_config: CaptureLandingRetargetConfig {
                activation_margin_m: capture_landing_activation_margin_m,
                full_scale_margin_m: capture_landing_full_scale_margin_m,
                maximum_authored_offset_m: capture_landing_maximum_authored_offset_m,
                maximum_root_to_landing_reach_m: capture_landing_maximum_root_reach_m,
                maximum_anchor_speed_mps: capture_landing_maximum_anchor_speed_mps,
            },
            capture_landing_freeze_ticks,
            reference_phase_retiming_enabled: touchdown_phase_retiming_enabled
                || balance_phase_retiming_enabled,
            touchdown_phase_retiming_enabled,
            balance_phase_retiming_enabled,
            balance_phase_hold_margin_m,
            balance_phase_full_rate_margin_m,
            touchdown_phase_retiming_config: TouchdownPhaseRetimingConfig {
                minimum_phase_rate: touchdown_phase_minimum_rate,
                maximum_phase_rate: 1.0,
                maximum_phase_rate_increase_per_tick: if touchdown_phase_release_ticks == 0 {
                    1.0
                } else {
                    1.0 / touchdown_phase_release_ticks as f64
                },
                maximum_phase_rate_decrease_per_tick: if touchdown_phase_engagement_ticks == 0 {
                    1.0
                } else {
                    1.0 / touchdown_phase_engagement_ticks as f64
                },
                guard_time_seconds: touchdown_phase_guard_time_seconds,
            },
            reference_phase: 0.0,
            reference_phase_rate: 1.0,
            previous_reference_phase_rate: 1.0,
            last_dcm_world: Vec3::zeros(),
            last_dcm_support_margin_m: f64::NAN,
            last_dcm_observation_valid: false,
            centroidal_angular_momentum_weight,
            centroidal_angular_momentum_priority,
            centroidal_angular_momentum_omega: std::f64::consts::TAU
                * centroidal_angular_momentum_frequency_hz,
            supported_weight,
            program,
        })
    }

    #[getter]
    fn dof(&self) -> usize {
        self.program.model.dof
    }

    #[getter]
    fn generalized_dof(&self) -> usize {
        self.program.model.dof + 6
    }

    /// Compute `M(q) * (observed_delta - predicted_delta)` for any compiled
    /// floating model. This is a model query, not contact or command authority.
    #[allow(clippy::too_many_arguments)]
    fn model_generalized_momentum_impulse_residuals(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        observed_delta_velocity: PyReadonlyArray1<'_, f64>,
        predicted_delta_velocity: PyReadonlyArray2<'_, f64>,
        mut residual_out: PyReadwriteArray2<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let predicted_shape = predicted_delta_velocity.as_array().dim();
        let residual_shape = residual_out.as_array().dim();
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let observed_delta_velocity = observed_delta_velocity.as_slice()?;
        let predicted_delta_velocity = predicted_delta_velocity.as_slice()?;
        let residual_out = residual_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || observed_delta_velocity.len() != generalized_dof
            || predicted_shape.0 == 0
            || predicted_shape.1 != generalized_dof
            || residual_shape != predicted_shape
        {
            return Err(PyValueError::new_err(format!(
                "generalized momentum residual expects root[3], quaternion[4], q[{dof}], observed[{generalized_dof}], predicted[C,{generalized_dof}], and residual[C,{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .chain(observed_delta_velocity)
            .chain(predicted_delta_velocity)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "generalized momentum residual inputs must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| {
            PyValueError::new_err("generalized momentum residual quaternion is degenerate")
        })?;
        self.state.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.state.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_momentum_impulse_residuals(
            &self.program.model,
            &self.state.robot,
            observed_delta_velocity,
            predicted_delta_velocity,
            &mut self.contact_transition_response_scratch,
            residual_out,
        )
        .map_err(|error| {
            PyValueError::new_err(format!("invalid generalized momentum residual: {error:?}"))
        })?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_momentum_impulse_residuals(
            &self.program.model,
            &self.state.robot,
            observed_delta_velocity,
            predicted_delta_velocity,
            &mut self.contact_transition_response_scratch,
            residual_out,
        )
        .expect("validated generalized momentum residual");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "generalized momentum residual allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    /// Map a generalized-momentum covector box through the exact full inverse
    /// mass for any compiled floating model. Component order is
    /// `[root angular; root linear; joints]` for both input and output.
    #[allow(clippy::too_many_arguments)]
    fn model_generalized_velocity_interval_from_momentum_box(
        &mut self,
        root_position: PyReadonlyArray1<'_, f64>,
        root_quaternion_wxyz: PyReadonlyArray1<'_, f64>,
        q: PyReadonlyArray1<'_, f64>,
        momentum_lower: PyReadonlyArray1<'_, f64>,
        momentum_upper: PyReadonlyArray1<'_, f64>,
        mut velocity_lower_out: PyReadwriteArray1<'_, f64>,
        mut velocity_upper_out: PyReadwriteArray1<'_, f64>,
    ) -> PyResult<(u64, u64, u64)> {
        let root_position = root_position.as_slice()?;
        let root_quaternion_wxyz = root_quaternion_wxyz.as_slice()?;
        let q = q.as_slice()?;
        let momentum_lower = momentum_lower.as_slice()?;
        let momentum_upper = momentum_upper.as_slice()?;
        let velocity_lower_out = velocity_lower_out.as_slice_mut()?;
        let velocity_upper_out = velocity_upper_out.as_slice_mut()?;
        let dof = self.program.model.dof;
        let generalized_dof = dof + 6;
        if root_position.len() != 3
            || root_quaternion_wxyz.len() != 4
            || q.len() != dof
            || momentum_lower.len() != generalized_dof
            || momentum_upper.len() != generalized_dof
            || velocity_lower_out.len() != generalized_dof
            || velocity_upper_out.len() != generalized_dof
        {
            return Err(PyValueError::new_err(format!(
                "momentum box expects root[3], quaternion[4], q[{dof}], momentum lower/upper[{generalized_dof}], and velocity lower/upper[{generalized_dof}]"
            )));
        }
        if root_position
            .iter()
            .chain(root_quaternion_wxyz)
            .chain(q)
            .any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "momentum-box model state must be finite",
            ));
        }
        let rotation = UnitQuaternion::try_new(
            nalgebra::Quaternion::new(
                root_quaternion_wxyz[0],
                root_quaternion_wxyz[1],
                root_quaternion_wxyz[2],
                root_quaternion_wxyz[3],
            ),
            1.0e-12,
        )
        .ok_or_else(|| PyValueError::new_err("momentum-box quaternion is degenerate"))?;
        self.state.robot.control_world_from_root = Transform3::from_parts(
            Translation3::new(root_position[0], root_position[1], root_position[2]),
            rotation,
        );
        self.state.robot.q.as_mut_slice().copy_from_slice(q);
        write_generalized_velocity_interval_from_momentum_box(
            &self.program.model,
            &self.state.robot,
            momentum_lower,
            momentum_upper,
            &mut self.contact_transition_response_scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .map_err(|error| PyValueError::new_err(format!("invalid momentum box: {error:?}")))?;

        let allocation_before = allocation_snapshot();
        let started = Instant::now();
        write_generalized_velocity_interval_from_momentum_box(
            &self.program.model,
            &self.state.robot,
            momentum_lower,
            momentum_upper,
            &mut self.contact_transition_response_scratch,
            velocity_lower_out,
            velocity_upper_out,
        )
        .expect("validated generalized momentum box");
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let allocation_after = allocation_snapshot();
        if allocation_after != allocation_before {
            return Err(PyValueError::new_err(
                "generalized momentum box allocated inside the Rust hot path",
            ));
        }
        Ok((
            elapsed_ns,
            allocation_after.0 - allocation_before.0,
            allocation_after.1 - allocation_before.1,
        ))
    }

    #[getter]
    fn task_diagnostic_capacity(&self) -> usize {
        FLOATING_TASK_DIAGNOSTIC_CAPACITY
    }

    #[getter]
    fn joint_names(&self) -> Vec<&str> {
        self.program.model.coordinate_names()
    }

    #[getter]
    fn joint_position_lower_limits(&self) -> Vec<f64> {
        let mut limits = vec![f64::NEG_INFINITY; self.program.model.dof];
        for joint in &self.program.model.joints {
            if let Some(coordinate) = joint.coordinate {
                limits[coordinate] = joint.limit.lower;
            }
        }
        limits
    }

    #[getter]
    fn joint_position_upper_limits(&self) -> Vec<f64> {
        let mut limits = vec![f64::INFINITY; self.program.model.dof];
        for joint in &self.program.model.joints {
            if let Some(coordinate) = joint.coordinate {
                limits[coordinate] = joint.limit.upper;
            }
        }
        limits
    }

    #[getter]
    fn joint_velocity_limits(&self) -> Vec<f64> {
        self.joint_velocity_limits.clone()
    }

    #[getter]
    fn actuator_effort_limits(&self) -> Vec<f64> {
        let bounds = if self.coupled_actuation_enabled {
            &self.actuator_effort_bounds
        } else {
            &self.torque_bounds
        };
        bounds.upper.iter().copied().collect()
    }

    #[getter]
    fn coupled_actuation_enabled(&self) -> bool {
        self.coupled_actuation_enabled
    }

    /// Copy, rather than share, another session's terminal hard-feasibility
    /// witness. The next solve must still match the complete Rust-owned key;
    /// every soft hierarchy and equality factorization remains destination
    /// local. This setup/tick boundary performs no allocation for compatible
    /// sessions compiled with the same fixed capacities.
    fn import_hard_feasibility_witness_from(
        &mut self,
        source: PyRef<'_, FloatingWbcSession>,
        mut diagnostics_out: PyReadwriteArray1<'_, u64>,
    ) -> PyResult<bool> {
        if diagnostics_out.len()? != 3 {
            return Err(PyValueError::new_err(
                "hard-feasibility witness diagnostics must have length three",
            ));
        }
        let before = allocation_snapshot();
        let started = Instant::now();
        let imported = self
            .scratch
            .import_hard_feasibility_witness_from(&source.scratch);
        let elapsed_ns = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let after = allocation_snapshot();
        diagnostics_out.as_slice_mut()?.copy_from_slice(&[
            elapsed_ns,
            after.0.saturating_sub(before.0),
            after.1.saturating_sub(before.1),
        ]);
        Ok(imported)
    }

    /// Replace the identity URDF fallback with a square coupled transmission.
    /// This is setup-time only: the inverse, validation, bounds, and solver
    /// capacity are all prepared before any trace enters the allocation-free
    /// tick loop.
    fn configure_coupled_actuation(
        &mut self,
        generalized_from_actuator: PyReadonlyArray2<'_, f64>,
        actuator_effort_limits: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        let map = generalized_from_actuator.as_array();
        let limits = actuator_effort_limits.as_slice()?;
        let dof = self.program.model.dof;
        if map.shape() != [dof, dof] || limits.len() != dof {
            return Err(PyValueError::new_err(format!(
                "coupled actuation requires map shape [{dof}, {dof}] and {dof} effort limits"
            )));
        }
        if map.iter().any(|value| !value.is_finite())
            || limits
                .iter()
                .any(|limit| !limit.is_finite() || *limit <= 0.0)
        {
            return Err(PyValueError::new_err(
                "coupled actuation map must be finite and effort limits finite and positive",
            ));
        }
        let forward = DMatrix::from_fn(dof, dof, |row, column| map[[row, column]]);
        let reverse = forward.clone().try_inverse().ok_or_else(|| {
            PyValueError::new_err("coupled actuation map must be square and invertible")
        })?;
        self.program.actuation.generalized_from_actuator = forward;
        self.program.actuation.actuator_from_generalized = Some(reverse);
        for (actuator, limit) in self
            .program
            .actuation
            .actuators
            .iter_mut()
            .zip(limits.iter().copied())
        {
            actuator.limits.effort = limit;
        }
        self.program
            .actuation
            .validate(&self.program.model)
            .map_err(value_error)?;
        for (coordinate, authored_limit) in limits.iter().copied().enumerate() {
            self.torque_bounds.lower[coordinate] = -self.maximum_generalized_torque;
            self.torque_bounds.upper[coordinate] = self.maximum_generalized_torque;
            let limit = authored_limit.min(self.maximum_generalized_torque);
            self.actuator_effort_bounds.lower[coordinate] = -limit;
            self.actuator_effort_bounds.upper[coordinate] = limit;
        }
        self.nominal_torque_bounds
            .lower
            .copy_from(&self.torque_bounds.lower);
        self.nominal_torque_bounds
            .upper
            .copy_from(&self.torque_bounds.upper);
        self.nominal_actuator_effort_bounds
            .lower
            .copy_from(&self.actuator_effort_bounds.lower);
        self.nominal_actuator_effort_bounds
            .upper
            .copy_from(&self.actuator_effort_bounds.upper);
        self.coupled_actuation_enabled = true;
        Ok(())
    }

    #[getter]
    fn maximum_touchdown_position_error_m(&self) -> f64 {
        self.support_transition_config
            .maximum_touchdown_position_error_m
    }

    #[getter]
    fn maximum_touchdown_tangential_speed_mps(&self) -> f64 {
        self.support_transition_config
            .maximum_touchdown_tangential_speed_mps
    }

    #[getter]
    fn maximum_touchdown_normal_speed_mps(&self) -> f64 {
        self.support_transition_config
            .maximum_touchdown_normal_speed_mps
    }

    #[getter]
    fn frame_names(&self) -> Vec<&str> {
        self.program
            .model
            .bodies
            .iter()
            .map(|body| body.name.as_str())
            .collect()
    }

    #[pyo3(signature = (q, v, root_translation_world, root_twist_world=None))]
    fn reset(
        &mut self,
        q: PyReadonlyArray1<'_, f64>,
        v: PyReadonlyArray1<'_, f64>,
        root_translation_world: PyReadonlyArray1<'_, f64>,
        root_twist_world: Option<PyReadonlyArray1<'_, f64>>,
    ) -> PyResult<()> {
        let q = q.as_slice()?;
        let v = v.as_slice()?;
        let root_translation_world = root_translation_world.as_slice()?;
        if q.len() != self.dof() || v.len() != self.dof() || root_translation_world.len() != 3 {
            return Err(PyValueError::new_err(
                "floating reset expects q[dof], v[dof], and root_translation[3]",
            ));
        }
        let mut state = FloatingRobotState::zeros(&self.program.model);
        state.robot.q.as_mut_slice().copy_from_slice(q);
        state.robot.v.as_mut_slice().copy_from_slice(v);
        state.robot.control_world_from_root.translation.vector =
            Vec3::from_row_slice(root_translation_world);
        if let Some(root_twist_world) = root_twist_world {
            let root_twist_world = root_twist_world.as_slice()?;
            if root_twist_world.len() != 6 {
                return Err(PyValueError::new_err(
                    "floating reset root twist must have six coordinates",
                ));
            }
            state
                .root_twist_world
                .0
                .as_mut_slice()
                .copy_from_slice(root_twist_world);
        }
        state.validate(&self.program.model).map_err(value_error)?;
        self.state = state;
        self.program
            .model
            .forward_kinematics(&self.state.robot, &mut self.tracking_cache)
            .map_err(value_error)?;
        self.previous_center_of_mass_world = self.tracking_cache.center_of_mass_world;
        self.support_transitions
            .fill(SupportTransitionState::default());
        self.contact_release_suppressed.fill(false);
        self.no_contact_safe_mode = false;
        self.precontact_authored_anchor_world.fill(Vec3::zeros());
        self.precontact_anchor_world.fill(Vec3::zeros());
        self.precontact_future_root_world.fill(Vec3::zeros());
        self.precontact_rotation_world
            .fill(UnitQuaternion::identity());
        self.precontact_planned.fill(false);
        self.contact_anchor_world.fill(Vec3::zeros());
        self.contact_patch_anchor_world.fill([Vec3::zeros(); 4]);
        self.contact_phase_authority_scale = self
            .contact_phase_authority_config
            .unsupported_joint_velocity_scale;
        self.last_dcm_world = Vec3::zeros();
        self.last_dcm_support_margin_m = f64::NAN;
        self.last_dcm_observation_valid = false;
        self.reference_phase = 0.0;
        self.reference_phase_rate = 1.0;
        self.previous_reference_phase_rate = 1.0;
        Ok(())
    }

    /// Solve every supplied witness tick independently at its oracle state.
    ///
    /// This path never calls the model integrator and contains no phase,
    /// capture, contact-admission, or feedback policy. It asks only whether the
    /// floating inverse-dynamics WBC can admit the morphology-projected
    /// acceleration jet at each state. Authored-to-projected residuals are
    /// scored separately by the caller before this dynamics boundary. When
    /// both optional realization arrays are present, the same state/contact
    /// construction instead imposes actuator effort as an exact equality and
    /// minimizes departure from the supplied admitted acceleration. That path
    /// remains state-local: it does not integrate or advance contact policy.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (
        root_positions,
        root_velocities,
        root_accelerations,
        q,
        v,
        joint_accelerations,
        frame_ids,
        contact_active,
        priorities,
        weights,
        root_angular_priority,
        root_height_priority,
        generalized_acceleration_out,
        actuator_torque_out,
        contact_normal_force_out,
        task_rms_out,
        task_clipped_out,
        dynamics_residual_out,
        contact_residual_out,
        minimum_friction_margin_out,
        minimum_support_margin_out,
        minimum_torque_margin_out,
        maximum_constraint_violation_out,
        minimum_bound_margin_out,
        minimum_joint_margin_rad_out,
        minimum_joint_headroom_fraction_out,
        limiting_joint_out,
        maximum_torque_utilization_out,
        minimum_torque_headroom_out,
        limiting_actuator_out,
        witness_acceleration_rms_out,
        step_ns_out,
        status_out,
        task_pseudoinverse_calls_out,
        task_pseudoinverse_calls_by_priority_out,
        clipped_steps_out,
        clipped_steps_by_priority_out,
        task_jacobi_sweeps_out,
        task_jacobi_sweeps_by_priority_out,
        feasibility_projection_sweeps_out,
        feasibility_halfspace_projections_out,
        feasibility_polish_iterations_out,
        allocation_calls_out,
        allocated_bytes_out,
        fixed_actuator_effort=None,
        realization_reference_acceleration=None,
        contact_force_basis_out=None,
        realization_reference_contact_force_basis=None,
        contact_modes=None,
        rolling_coordinates=None,
        rolling_velocity_coefficients=None,
        rolling_velocity_stabilization_gains=None,
        rolling_maximum_stabilization_accelerations=None,
        contact_mode_trace=None,
        generalized_acceleration_limit_scales=None,
        actuator_effort_limit_scales=None,
        root_quaternions_wxyz=None,
        root_angular_velocities_world=None,
        root_angular_accelerations_world=None,
        contact_bases_world=None,
        feasibility_seed_reused_out=None,
        feasibility_prefix_resumed_out=None
    ))]
    fn run_oracle_trace(
        &mut self,
        root_positions: PyReadonlyArray2<'_, f64>,
        root_velocities: PyReadonlyArray2<'_, f64>,
        root_accelerations: PyReadonlyArray2<'_, f64>,
        q: PyReadonlyArray2<'_, f64>,
        v: PyReadonlyArray2<'_, f64>,
        joint_accelerations: PyReadonlyArray2<'_, f64>,
        frame_ids: PyReadonlyArray1<'_, i64>,
        contact_active: PyReadonlyArray2<'_, u8>,
        priorities: PyReadonlyArray1<'_, u8>,
        weights: PyReadonlyArray1<'_, f64>,
        root_angular_priority: u8,
        root_height_priority: u8,
        mut generalized_acceleration_out: PyReadwriteArray2<'_, f64>,
        mut actuator_torque_out: PyReadwriteArray2<'_, f64>,
        mut contact_normal_force_out: PyReadwriteArray2<'_, f64>,
        mut task_rms_out: PyReadwriteArray2<'_, f64>,
        mut task_clipped_out: PyReadwriteArray2<'_, u8>,
        mut dynamics_residual_out: PyReadwriteArray1<'_, f64>,
        mut contact_residual_out: PyReadwriteArray1<'_, f64>,
        mut minimum_friction_margin_out: PyReadwriteArray1<'_, f64>,
        mut minimum_support_margin_out: PyReadwriteArray1<'_, f64>,
        mut minimum_torque_margin_out: PyReadwriteArray1<'_, f64>,
        mut maximum_constraint_violation_out: PyReadwriteArray1<'_, f64>,
        mut minimum_bound_margin_out: PyReadwriteArray1<'_, f64>,
        mut minimum_joint_margin_rad_out: PyReadwriteArray1<'_, f64>,
        mut minimum_joint_headroom_fraction_out: PyReadwriteArray1<'_, f64>,
        mut limiting_joint_out: PyReadwriteArray1<'_, u16>,
        mut maximum_torque_utilization_out: PyReadwriteArray1<'_, f64>,
        mut minimum_torque_headroom_out: PyReadwriteArray1<'_, f64>,
        mut limiting_actuator_out: PyReadwriteArray1<'_, u16>,
        mut witness_acceleration_rms_out: PyReadwriteArray1<'_, f64>,
        mut step_ns_out: PyReadwriteArray1<'_, u64>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut task_pseudoinverse_calls_out: PyReadwriteArray1<'_, u16>,
        mut task_pseudoinverse_calls_by_priority_out: PyReadwriteArray2<'_, u16>,
        mut clipped_steps_out: PyReadwriteArray1<'_, u16>,
        mut clipped_steps_by_priority_out: PyReadwriteArray2<'_, u16>,
        mut task_jacobi_sweeps_out: PyReadwriteArray1<'_, u16>,
        mut task_jacobi_sweeps_by_priority_out: PyReadwriteArray2<'_, u16>,
        mut feasibility_projection_sweeps_out: PyReadwriteArray1<'_, u16>,
        mut feasibility_halfspace_projections_out: PyReadwriteArray1<'_, u32>,
        mut feasibility_polish_iterations_out: PyReadwriteArray1<'_, u16>,
        mut allocation_calls_out: PyReadwriteArray1<'_, u64>,
        mut allocated_bytes_out: PyReadwriteArray1<'_, u64>,
        fixed_actuator_effort: Option<PyReadonlyArray2<'_, f64>>,
        realization_reference_acceleration: Option<PyReadonlyArray2<'_, f64>>,
        mut contact_force_basis_out: Option<PyReadwriteArray3<'_, f64>>,
        realization_reference_contact_force_basis: Option<PyReadonlyArray3<'_, f64>>,
        contact_modes: Option<PyReadonlyArray1<'_, u8>>,
        rolling_coordinates: Option<PyReadonlyArray1<'_, i64>>,
        rolling_velocity_coefficients: Option<PyReadonlyArray1<'_, f64>>,
        rolling_velocity_stabilization_gains: Option<PyReadonlyArray1<'_, f64>>,
        rolling_maximum_stabilization_accelerations: Option<PyReadonlyArray1<'_, f64>>,
        contact_mode_trace: Option<PyReadonlyArray2<'_, u8>>,
        generalized_acceleration_limit_scales: Option<PyReadonlyArray2<'_, f64>>,
        actuator_effort_limit_scales: Option<PyReadonlyArray2<'_, f64>>,
        root_quaternions_wxyz: Option<PyReadonlyArray2<'_, f64>>,
        root_angular_velocities_world: Option<PyReadonlyArray2<'_, f64>>,
        root_angular_accelerations_world: Option<PyReadonlyArray2<'_, f64>>,
        contact_bases_world: Option<PyReadonlyArray4<'_, f64>>,
        mut feasibility_seed_reused_out: Option<PyReadwriteArray1<'_, u8>>,
        mut feasibility_prefix_resumed_out: Option<PyReadwriteArray1<'_, u8>>,
    ) -> PyResult<()> {
        // Every call begins from the configured nominal authority. This also
        // self-heals a session after any model error returned from the middle
        // of an earlier trace.
        self.acceleration_bounds
            .lower
            .copy_from(&self.nominal_acceleration_bounds.lower);
        self.acceleration_bounds
            .upper
            .copy_from(&self.nominal_acceleration_bounds.upper);
        self.torque_bounds
            .lower
            .copy_from(&self.nominal_torque_bounds.lower);
        self.torque_bounds
            .upper
            .copy_from(&self.nominal_torque_bounds.upper);
        self.actuator_effort_bounds
            .lower
            .copy_from(&self.nominal_actuator_effort_bounds.lower);
        self.actuator_effort_bounds
            .upper
            .copy_from(&self.nominal_actuator_effort_bounds.upper);
        let root_positions = root_positions.as_array();
        let root_velocities = root_velocities.as_array();
        let root_accelerations = root_accelerations.as_array();
        let q = q.as_array();
        let v = v.as_array();
        let joint_accelerations = joint_accelerations.as_array();
        let frame_ids = frame_ids.as_slice()?;
        let contact_active = contact_active.as_array();
        let priorities = priorities.as_slice()?;
        let weights = weights.as_slice()?;
        let mut generalized_acceleration_out = generalized_acceleration_out.as_array_mut();
        let mut actuator_torque_out = actuator_torque_out.as_array_mut();
        let mut contact_normal_force_out = contact_normal_force_out.as_array_mut();
        let mut task_rms_out = task_rms_out.as_array_mut();
        let mut task_clipped_out = task_clipped_out.as_array_mut();
        let dynamics_residual_out = dynamics_residual_out.as_slice_mut()?;
        let contact_residual_out = contact_residual_out.as_slice_mut()?;
        let minimum_friction_margin_out = minimum_friction_margin_out.as_slice_mut()?;
        let minimum_support_margin_out = minimum_support_margin_out.as_slice_mut()?;
        let minimum_torque_margin_out = minimum_torque_margin_out.as_slice_mut()?;
        let maximum_constraint_violation_out = maximum_constraint_violation_out.as_slice_mut()?;
        let minimum_bound_margin_out = minimum_bound_margin_out.as_slice_mut()?;
        let minimum_joint_margin_rad_out = minimum_joint_margin_rad_out.as_slice_mut()?;
        let minimum_joint_headroom_fraction_out =
            minimum_joint_headroom_fraction_out.as_slice_mut()?;
        let limiting_joint_out = limiting_joint_out.as_slice_mut()?;
        let maximum_torque_utilization_out = maximum_torque_utilization_out.as_slice_mut()?;
        let minimum_torque_headroom_out = minimum_torque_headroom_out.as_slice_mut()?;
        let limiting_actuator_out = limiting_actuator_out.as_slice_mut()?;
        let witness_acceleration_rms_out = witness_acceleration_rms_out.as_slice_mut()?;
        let step_ns_out = step_ns_out.as_slice_mut()?;
        let status_out = status_out.as_slice_mut()?;
        let task_pseudoinverse_calls_out = task_pseudoinverse_calls_out.as_slice_mut()?;
        let mut task_pseudoinverse_calls_by_priority_out =
            task_pseudoinverse_calls_by_priority_out.as_array_mut();
        let clipped_steps_out = clipped_steps_out.as_slice_mut()?;
        let mut clipped_steps_by_priority_out = clipped_steps_by_priority_out.as_array_mut();
        let task_jacobi_sweeps_out = task_jacobi_sweeps_out.as_slice_mut()?;
        let mut task_jacobi_sweeps_by_priority_out =
            task_jacobi_sweeps_by_priority_out.as_array_mut();
        let feasibility_projection_sweeps_out = feasibility_projection_sweeps_out.as_slice_mut()?;
        let feasibility_halfspace_projections_out =
            feasibility_halfspace_projections_out.as_slice_mut()?;
        let feasibility_polish_iterations_out = feasibility_polish_iterations_out.as_slice_mut()?;
        let allocation_calls_out = allocation_calls_out.as_slice_mut()?;
        let allocated_bytes_out = allocated_bytes_out.as_slice_mut()?;
        let fixed_actuator_effort = fixed_actuator_effort
            .as_ref()
            .map(PyReadonlyArray2::as_array);
        let realization_reference_acceleration = realization_reference_acceleration
            .as_ref()
            .map(PyReadonlyArray2::as_array);
        let mut contact_force_basis_out = contact_force_basis_out
            .as_mut()
            .map(PyReadwriteArray3::as_array_mut);
        let realization_reference_contact_force_basis = realization_reference_contact_force_basis
            .as_ref()
            .map(PyReadonlyArray3::as_array);
        let contact_modes = contact_modes
            .as_ref()
            .map(|values| values.as_slice())
            .transpose()?;
        let rolling_coordinates = rolling_coordinates
            .as_ref()
            .map(|values| values.as_slice())
            .transpose()?;
        let rolling_velocity_coefficients = rolling_velocity_coefficients
            .as_ref()
            .map(|values| values.as_slice())
            .transpose()?;
        let rolling_velocity_stabilization_gains = rolling_velocity_stabilization_gains
            .as_ref()
            .map(|values| values.as_slice())
            .transpose()?;
        let rolling_maximum_stabilization_accelerations =
            rolling_maximum_stabilization_accelerations
                .as_ref()
                .map(|values| values.as_slice())
                .transpose()?;
        let contact_mode_trace = contact_mode_trace.as_ref().map(PyReadonlyArray2::as_array);
        let generalized_acceleration_limit_scales = generalized_acceleration_limit_scales
            .as_ref()
            .map(PyReadonlyArray2::as_array);
        let actuator_effort_limit_scales = actuator_effort_limit_scales
            .as_ref()
            .map(PyReadonlyArray2::as_array);
        let root_quaternions_wxyz = root_quaternions_wxyz
            .as_ref()
            .map(PyReadonlyArray2::as_array);
        let root_angular_velocities_world = root_angular_velocities_world
            .as_ref()
            .map(PyReadonlyArray2::as_array);
        let root_angular_accelerations_world = root_angular_accelerations_world
            .as_ref()
            .map(PyReadonlyArray2::as_array);
        let contact_bases_world = contact_bases_world.as_ref().map(PyReadonlyArray4::as_array);
        let mut feasibility_seed_reused_out = feasibility_seed_reused_out
            .as_mut()
            .map(|values| values.as_slice_mut())
            .transpose()?;
        let mut feasibility_prefix_resumed_out = feasibility_prefix_resumed_out
            .as_mut()
            .map(|values| values.as_slice_mut())
            .transpose()?;
        let ticks = root_positions.shape()[0];
        let target_count = frame_ids.len();
        let dof = self.dof();
        let generalized_dof = dof + 6;
        if target_count > FLOATING_POINT_TASK_CAPACITY
            || root_positions.shape() != [ticks, 3]
            || root_velocities.shape() != [ticks, 3]
            || root_accelerations.shape() != [ticks, 3]
            || q.shape() != [ticks, dof]
            || v.shape() != [ticks, dof]
            || joint_accelerations.shape() != [ticks, dof]
            || contact_active.shape() != [ticks, target_count]
            || priorities.len() != target_count
            || weights.len() != target_count
            || generalized_acceleration_out.shape() != [ticks, generalized_dof]
            || actuator_torque_out.shape() != [ticks, dof]
            || contact_normal_force_out.shape() != [ticks, self.maximum_contacts]
            || task_rms_out.shape() != [ticks, FLOATING_TASK_DIAGNOSTIC_CAPACITY]
            || task_clipped_out.shape() != [ticks, FLOATING_TASK_DIAGNOSTIC_CAPACITY]
            || dynamics_residual_out.len() != ticks
            || contact_residual_out.len() != ticks
            || minimum_friction_margin_out.len() != ticks
            || minimum_support_margin_out.len() != ticks
            || minimum_torque_margin_out.len() != ticks
            || maximum_constraint_violation_out.len() != ticks
            || minimum_bound_margin_out.len() != ticks
            || minimum_joint_margin_rad_out.len() != ticks
            || minimum_joint_headroom_fraction_out.len() != ticks
            || limiting_joint_out.len() != ticks
            || maximum_torque_utilization_out.len() != ticks
            || minimum_torque_headroom_out.len() != ticks
            || limiting_actuator_out.len() != ticks
            || witness_acceleration_rms_out.len() != ticks
            || step_ns_out.len() != ticks
            || status_out.len() != ticks
            || task_pseudoinverse_calls_out.len() != ticks
            || task_pseudoinverse_calls_by_priority_out.shape() != [ticks, Priority::ALL.len()]
            || clipped_steps_out.len() != ticks
            || clipped_steps_by_priority_out.shape() != [ticks, Priority::ALL.len()]
            || task_jacobi_sweeps_out.len() != ticks
            || task_jacobi_sweeps_by_priority_out.shape() != [ticks, Priority::ALL.len()]
            || feasibility_projection_sweeps_out.len() != ticks
            || feasibility_halfspace_projections_out.len() != ticks
            || feasibility_polish_iterations_out.len() != ticks
            || allocation_calls_out.len() != ticks
            || allocated_bytes_out.len() != ticks
            || feasibility_seed_reused_out
                .as_ref()
                .is_some_and(|values| values.len() != ticks)
            || feasibility_prefix_resumed_out
                .as_ref()
                .is_some_and(|values| values.len() != ticks)
            || fixed_actuator_effort
                .as_ref()
                .is_some_and(|effort| effort.shape() != [ticks, dof])
            || realization_reference_acceleration
                .as_ref()
                .is_some_and(|acceleration| acceleration.shape() != [ticks, generalized_dof])
            || fixed_actuator_effort.is_some() != realization_reference_acceleration.is_some()
            || contact_force_basis_out
                .as_ref()
                .is_some_and(|forces| forces.shape() != [ticks, self.maximum_contacts, 3])
            || realization_reference_contact_force_basis
                .as_ref()
                .is_some_and(|forces| forces.shape() != [ticks, self.maximum_contacts, 3])
            || (realization_reference_contact_force_basis.is_some()
                && fixed_actuator_effort.is_none())
            || contact_modes.is_some_and(|values| values.len() != target_count)
            || contact_mode_trace
                .as_ref()
                .is_some_and(|values| values.shape() != [ticks, target_count])
            || (contact_modes.is_some() && contact_mode_trace.is_some())
            || generalized_acceleration_limit_scales
                .as_ref()
                .is_some_and(|scales| scales.shape() != [ticks, generalized_dof])
            || actuator_effort_limit_scales
                .as_ref()
                .is_some_and(|scales| scales.shape() != [ticks, dof])
            || root_quaternions_wxyz
                .as_ref()
                .is_some_and(|quaternions| quaternions.shape() != [ticks, 4])
            || root_angular_velocities_world
                .as_ref()
                .is_some_and(|velocities| velocities.shape() != [ticks, 3])
            || root_angular_accelerations_world
                .as_ref()
                .is_some_and(|accelerations| accelerations.shape() != [ticks, 3])
            || contact_bases_world
                .as_ref()
                .is_some_and(|bases| bases.shape() != [ticks, target_count, 3, 3])
            || rolling_coordinates.is_some_and(|values| values.len() != target_count)
            || rolling_velocity_coefficients.is_some_and(|values| values.len() != target_count)
            || rolling_velocity_stabilization_gains
                .is_some_and(|values| values.len() != target_count)
            || rolling_maximum_stabilization_accelerations
                .is_some_and(|values| values.len() != target_count)
        {
            return Err(PyValueError::new_err("oracle WBC array shape mismatch"));
        }
        let has_rolling_wheel = contact_modes.is_some_and(|modes| modes.contains(&3))
            || contact_mode_trace
                .as_ref()
                .is_some_and(|modes| modes.iter().any(|mode| *mode == 3));
        let has_contact_mode_descriptor = contact_modes.is_some() || contact_mode_trace.is_some();
        let has_complete_rolling_descriptor = rolling_coordinates.is_some()
            && rolling_velocity_coefficients.is_some()
            && rolling_velocity_stabilization_gains.is_some()
            && rolling_maximum_stabilization_accelerations.is_some();
        if frame_ids
            .iter()
            .any(|&frame| frame < 0 || frame as usize >= self.program.model.bodies.len())
            || weights
                .iter()
                .any(|weight| !weight.is_finite() || *weight < 0.0)
            || root_positions.iter().any(|value| !value.is_finite())
            || root_velocities.iter().any(|value| !value.is_finite())
            || root_accelerations.iter().any(|value| !value.is_finite())
            || q.iter().any(|value| !value.is_finite())
            || v.iter().any(|value| !value.is_finite())
            || joint_accelerations.iter().any(|value| !value.is_finite())
            || root_quaternions_wxyz
                .as_ref()
                .is_some_and(|quaternions| quaternions.iter().any(|value| !value.is_finite()))
            || root_angular_velocities_world
                .as_ref()
                .is_some_and(|velocities| velocities.iter().any(|value| !value.is_finite()))
            || root_angular_accelerations_world
                .as_ref()
                .is_some_and(|accelerations| accelerations.iter().any(|value| !value.is_finite()))
            || contact_bases_world.as_ref().is_some_and(|bases| {
                (0..ticks).any(|tick| {
                    (0..target_count).any(|target| {
                        let axis = |basis: usize| {
                            Vec3::new(
                                bases[[tick, target, basis, 0]],
                                bases[[tick, target, basis, 1]],
                                bases[[tick, target, basis, 2]],
                            )
                        };
                        let tangent_x = axis(0);
                        let tangent_y = axis(1);
                        let normal = axis(2);
                        !tangent_x.iter().all(|value| value.is_finite())
                            || !tangent_y.iter().all(|value| value.is_finite())
                            || !normal.iter().all(|value| value.is_finite())
                            || (tangent_x.norm() - 1.0).abs() > 1.0e-8
                            || (tangent_y.norm() - 1.0).abs() > 1.0e-8
                            || (normal.norm() - 1.0).abs() > 1.0e-8
                            || tangent_x.dot(&tangent_y).abs() > 1.0e-8
                            || tangent_x.dot(&normal).abs() > 1.0e-8
                            || tangent_y.dot(&normal).abs() > 1.0e-8
                            || tangent_x.cross(&tangent_y).dot(&normal) < 1.0 - 1.0e-8
                    })
                })
            })
            || generalized_acceleration_limit_scales
                .as_ref()
                .is_some_and(|scales| {
                    scales
                        .iter()
                        .any(|scale| !scale.is_finite() || !(0.0..=1.0).contains(scale))
                })
            || actuator_effort_limit_scales.as_ref().is_some_and(|scales| {
                scales
                    .iter()
                    .any(|scale| !scale.is_finite() || !(0.0..=1.0).contains(scale))
            })
            || contact_active.iter().any(|active| *active > 1)
            || fixed_actuator_effort.as_ref().is_some_and(|effort| {
                effort.indexed_iter().any(|((tick, coordinate), value)| {
                    let scale = actuator_effort_limit_scales
                        .as_ref()
                        .map_or(1.0, |scales| scales[[tick, coordinate]]);
                    !value.is_finite()
                        || *value < self.nominal_torque_bounds.lower[coordinate] * scale - 1e-12
                        || *value > self.nominal_torque_bounds.upper[coordinate] * scale + 1e-12
                })
            })
            || (fixed_actuator_effort.is_some() && self.coupled_actuation_enabled)
            || realization_reference_acceleration
                .as_ref()
                .is_some_and(|acceleration| acceleration.iter().any(|value| !value.is_finite()))
            || realization_reference_contact_force_basis
                .as_ref()
                .is_some_and(|forces| forces.iter().any(|value| !value.is_finite()))
            || contact_modes.is_some_and(|modes| modes.iter().any(|mode| *mode > 3))
            || contact_mode_trace
                .as_ref()
                .is_some_and(|modes| modes.iter().any(|mode| *mode > 3))
            || (has_contact_mode_descriptor && self.contact_points_per_target != 1)
            || has_rolling_wheel != has_complete_rolling_descriptor
            || (!has_contact_mode_descriptor && has_complete_rolling_descriptor)
            || (0..target_count).any(|target| {
                let uses_rolling_wheel = contact_modes.is_some_and(|modes| modes[target] == 3)
                    || contact_mode_trace
                        .as_ref()
                        .is_some_and(|modes| (0..ticks).any(|tick| modes[[tick, target]] == 3));
                if !uses_rolling_wheel {
                    return false;
                }
                let coordinate = rolling_coordinates.expect("validated rolling descriptor")[target];
                let coefficient =
                    rolling_velocity_coefficients.expect("validated rolling descriptor")[target];
                let gain = rolling_velocity_stabilization_gains
                    .expect("validated rolling descriptor")[target];
                let maximum = rolling_maximum_stabilization_accelerations
                    .expect("validated rolling descriptor")[target];
                coordinate < 0
                    || coordinate as usize >= dof
                    || !coefficient.is_finite()
                    || coefficient.abs() <= 1e-12
                    || !gain.is_finite()
                    || gain < 0.0
                    || !maximum.is_finite()
                    || maximum < 0.0
            })
        {
            return Err(PyValueError::new_err(
                "oracle WBC frame, weight, contact mode, or rolling descriptor is invalid",
            ));
        }
        let point_priorities = priorities
            .iter()
            .copied()
            .map(priority)
            .collect::<PyResult<Vec<_>>>()?;
        let root_angular_priority = priority(root_angular_priority)?;
        let root_height_priority = priority(root_height_priority)?;
        for tick in 0..ticks {
            if let Some(scales) = generalized_acceleration_limit_scales.as_ref() {
                for coordinate in 0..generalized_dof {
                    let scale = scales[[tick, coordinate]];
                    self.acceleration_bounds.lower[coordinate] =
                        self.nominal_acceleration_bounds.lower[coordinate] * scale;
                    self.acceleration_bounds.upper[coordinate] =
                        self.nominal_acceleration_bounds.upper[coordinate] * scale;
                }
            }
            if let Some(scales) = actuator_effort_limit_scales.as_ref() {
                let (active, nominal) = if self.coupled_actuation_enabled {
                    (
                        &mut self.actuator_effort_bounds,
                        &self.nominal_actuator_effort_bounds,
                    )
                } else {
                    (&mut self.torque_bounds, &self.nominal_torque_bounds)
                };
                for coordinate in 0..dof {
                    let scale = scales[[tick, coordinate]];
                    active.lower[coordinate] = nominal.lower[coordinate] * scale;
                    active.upper[coordinate] = nominal.upper[coordinate] * scale;
                }
            }
            let (allocation_calls_before, allocated_bytes_before) = allocation_snapshot();
            let started = Instant::now();
            self.state.robot.control_world_from_root.rotation = if let Some(quaternions) =
                root_quaternions_wxyz.as_ref()
            {
                let quaternion = nalgebra::Quaternion::new(
                    quaternions[[tick, 0]],
                    quaternions[[tick, 1]],
                    quaternions[[tick, 2]],
                    quaternions[[tick, 3]],
                );
                UnitQuaternion::try_new(quaternion, 1.0e-12).ok_or_else(|| {
                    PyValueError::new_err("root_quaternions_wxyz contains a degenerate quaternion")
                })?
            } else {
                UnitQuaternion::identity()
            };
            for axis in 0..3 {
                self.state.robot.control_world_from_root.translation.vector[axis] =
                    root_positions[[tick, axis]];
                self.state.root_twist_world.0[axis] = root_angular_velocities_world
                    .as_ref()
                    .map_or(0.0, |velocities| velocities[[tick, axis]]);
                self.state.root_twist_world.0[3 + axis] = root_velocities[[tick, axis]];
                self.desired_acceleration[axis] = root_angular_accelerations_world
                    .as_ref()
                    .map_or(0.0, |accelerations| accelerations[[tick, axis]]);
                self.desired_acceleration[3 + axis] = root_accelerations[[tick, axis]];
            }
            for coordinate in 0..dof {
                self.state.robot.q[coordinate] = q[[tick, coordinate]];
                self.state.robot.v[coordinate] = v[[tick, coordinate]];
                self.desired_acceleration[6 + coordinate] = joint_accelerations[[tick, coordinate]];
            }
            if let Some(reference) = realization_reference_acceleration.as_ref() {
                for coordinate in 0..generalized_dof {
                    self.desired_acceleration[coordinate] = reference[[tick, coordinate]];
                }
            }
            self.state
                .validate(&self.program.model)
                .map_err(value_error)?;
            self.program
                .model
                .forward_kinematics(&self.state.robot, &mut self.tracking_cache)
                .map_err(value_error)?;
            self.program
                .model
                .floating_inverse_dynamics_into(
                    &self.state.robot,
                    self.state.root_twist_world,
                    &self.desired_acceleration,
                    Vec3::zeros(),
                    &self.tracking_cache,
                    &mut self.centroidal_dynamics,
                    &mut self.generalized_velocity,
                )
                .map_err(value_error)?;
            self.point_tasks.clear();
            self.angular_tasks.clear();
            self.contacts.clear();
            self.support_patches.clear();
            for target in 0..target_count {
                let frame = bonesaw_core::FrameId(frame_ids[target] as usize);
                self.program
                    .model
                    .floating_point_jacobian_into(
                        &self.tracking_cache,
                        frame,
                        Vec3::zeros(),
                        &mut self.tracking_jacobian,
                    )
                    .map_err(value_error)?;
                let bias = self
                    .program
                    .model
                    .point_bias_acceleration_world(
                        frame,
                        Vec3::zeros(),
                        &self.tracking_cache,
                        &self.centroidal_dynamics,
                    )
                    .map_err(value_error)?;
                let desired = bias
                    + Vec3::from_fn(|axis, _| {
                        (0..generalized_dof)
                            .map(|coordinate| {
                                self.tracking_jacobian[(axis, coordinate)]
                                    * self.desired_acceleration[coordinate]
                            })
                            .sum()
                    });
                let is_contact = contact_active[[tick, target]] != 0;
                self.point_tasks.push(FloatingPointAccelerationTask {
                    stable_id: 10 + target as u32,
                    frame,
                    point_in_frame: Vec3::zeros(),
                    desired_acceleration_world: desired,
                    priority: point_priorities[target],
                    weight: if is_contact { 0.0 } else { weights[target] },
                });
                if !is_contact && weights[target] > 0.0 {
                    self.angular_tasks
                        .push(FloatingFrameAngularAccelerationTask {
                            stable_id: 50 + target as u32,
                            frame,
                            desired_angular_acceleration_world: {
                                self.program
                                    .model
                                    .floating_angular_jacobian_into(
                                        &self.tracking_cache,
                                        frame,
                                        &mut self.tracking_jacobian,
                                    )
                                    .map_err(value_error)?;
                                self.program
                                    .model
                                    .angular_bias_acceleration_world(
                                        frame,
                                        &self.centroidal_dynamics,
                                    )
                                    .map_err(value_error)?
                                    + Vec3::from_fn(|axis, _| {
                                        (0..generalized_dof)
                                            .map(|coordinate| {
                                                self.tracking_jacobian[(axis, coordinate)]
                                                    * self.desired_acceleration[coordinate]
                                            })
                                            .sum()
                                    })
                            },
                            priority: point_priorities[target],
                            weight: weights[target],
                        });
                }
                if is_contact {
                    let first_contact = self.contacts.len();
                    for point in 0..self.contact_points_per_target {
                        let (mode, kinematic_enabled) = if self.contact_points_per_target == 4 {
                            match point {
                                0 => (ContactMode::LockedPoint, true),
                                1 => (ContactMode::NormalPoint, true),
                                2 => (ContactMode::LockedPoint, false),
                                3 => (ContactMode::RollingPoint, true),
                                _ => unreachable!(),
                            }
                        } else {
                            let mode_code = contact_mode_trace.as_ref().map_or_else(
                                || contact_modes.map_or(0, |modes| modes[target]),
                                |modes| modes[[tick, target]],
                            );
                            let mode = match mode_code {
                                0 => ContactMode::LockedPoint,
                                1 => ContactMode::NormalPoint,
                                2 => ContactMode::RollingPoint,
                                3 => ContactMode::RollingWheel {
                                    coordinate: rolling_coordinates
                                        .expect("validated rolling descriptor")[target]
                                        as usize,
                                    velocity_coefficient: rolling_velocity_coefficients
                                        .expect("validated rolling descriptor")[target],
                                    velocity_stabilization_gain:
                                        rolling_velocity_stabilization_gains
                                            .expect("validated rolling descriptor")[target],
                                    maximum_stabilization_acceleration:
                                        rolling_maximum_stabilization_accelerations
                                            .expect("validated rolling descriptor")[target],
                                },
                                _ => unreachable!("validated contact mode"),
                            };
                            (mode, true)
                        };
                        let mut contact = ContactSpec::horizontal(
                            1 + (target * 4 + point) as u32,
                            frame,
                            self.contact_patch_points[point],
                            mode,
                            self.contact_friction_coefficient,
                            self.maximum_normal_force_multiple * self.supported_weight,
                            0.0,
                        );
                        if let Some(bases) = contact_bases_world.as_ref() {
                            contact.tangent_x_world = Vec3::new(
                                bases[[tick, target, 0, 0]],
                                bases[[tick, target, 0, 1]],
                                bases[[tick, target, 0, 2]],
                            );
                            contact.tangent_y_world = Vec3::new(
                                bases[[tick, target, 1, 0]],
                                bases[[tick, target, 1, 1]],
                                bases[[tick, target, 1, 2]],
                            );
                            contact.normal_world = Vec3::new(
                                bases[[tick, target, 2, 0]],
                                bases[[tick, target, 2, 1]],
                                bases[[tick, target, 2, 2]],
                            );
                        }
                        contact.kinematic_enabled = kinematic_enabled;
                        // A material oracle contact is stationary in the
                        // control world. Contact rows consume physical point
                        // acceleration, so zero is the coherent target; the
                        // WBC subtracts J-dot-v internally.
                        contact.desired_point_acceleration_world = Vec3::zeros();
                        self.contacts.push(contact);
                    }
                    if self.contact_points_per_target == 4
                        && self.minimum_contact_cop_margin_m > 0.0
                    {
                        self.support_patches.push(SupportPatchSpec {
                            stable_id: 1 + target as u32,
                            first_contact,
                            contact_count: 4,
                            minimum_margin_m: self.minimum_contact_cop_margin_m,
                        });
                    }
                }
            }
            if self.contacts.len() > self.maximum_contacts {
                return Err(PyValueError::new_err(
                    "oracle WBC contact patch exceeds session capacity",
                ));
            }
            if !self.contacts.is_empty() {
                let nominal_normal_force = self.supported_weight / self.contacts.len() as f64;
                for (contact_index, contact) in self.contacts.iter_mut().enumerate() {
                    if let Some(reference) = realization_reference_contact_force_basis.as_ref() {
                        contact.nominal_tangent_x_force = reference[[tick, contact_index, 0]];
                        contact.nominal_tangent_y_force = reference[[tick, contact_index, 1]];
                        contact.nominal_normal_force = reference[[tick, contact_index, 2]];
                    } else {
                        contact.nominal_normal_force = nominal_normal_force;
                    }
                }
            }
            let center_of_mass_task = if self.center_of_mass_task_weight > 0.0 {
                self.program
                    .model
                    .floating_com_jacobian_into(
                        &self.tracking_cache,
                        &mut self.centroidal_dynamics,
                        &mut self.tracking_jacobian,
                    )
                    .map_err(value_error)?;
                let desired_acceleration_world = self
                    .program
                    .model
                    .center_of_mass_bias_acceleration_world(
                        &self.tracking_cache,
                        &self.centroidal_dynamics,
                    )
                    .map_err(value_error)?
                    + Vec3::from_fn(|axis, _| {
                        (0..generalized_dof)
                            .map(|coordinate| {
                                self.tracking_jacobian[(axis, coordinate)]
                                    * self.desired_acceleration[coordinate]
                            })
                            .sum()
                    });
                Some(FloatingCenterOfMassTask {
                    desired_acceleration_world,
                    // Keep the floating task basis independent: CoM owns
                    // horizontal balance while the semantic root-height row
                    // owns vertical motion. Asking both root Z and CoM Z at
                    // the same priority creates a redundant, morphology-
                    // dependent trade instead of measuring either behavior.
                    horizontal_only: true,
                    priority: self.center_of_mass_task_priority,
                    weight: self.center_of_mass_task_weight,
                })
            } else {
                None
            };
            let centroidal_angular_momentum_task = (self.centroidal_angular_momentum_weight > 0.0)
                .then_some(FloatingCentroidalAngularMomentumTask {
                    desired_rate_world: Vec3::zeros(),
                    priority: self.centroidal_angular_momentum_priority,
                    weight: self.centroidal_angular_momentum_weight,
                });
            if let Some(effort) = fixed_actuator_effort.as_ref() {
                for coordinate in 0..dof {
                    self.mapped_actuator_effort[coordinate] = effort[[tick, coordinate]];
                }
            }
            let solve_input = FloatingDynamicWbcInput {
                state: &self.state.robot,
                root_twist_world: self.state.root_twist_world,
                desired_generalized_acceleration: &self.desired_acceleration,
                task_priorities: FloatingTaskPriorities {
                    root_angular: root_angular_priority,
                    root_horizontal: self.root_horizontal_task_priority,
                    root_height: root_height_priority,
                    joint_posture: self.joint_posture_priority,
                },
                task_weights: FloatingTaskWeights {
                    root_angular: self.root_angular_task_weight,
                    root_horizontal: self.root_horizontal_task_weight,
                    root_height: self.root_height_task_weight,
                    joint_posture: 1.0,
                },
                joint_posture_weight: self.joint_posture_weight,
                joint_acceleration_task: None,
                center_of_mass_task,
                centroidal_angular_momentum_task,
                frame_angular_acceleration_tasks: &self.angular_tasks,
                point_acceleration_tasks: &self.point_tasks,
                generalized_acceleration_bounds: &self.acceleration_bounds,
                torque_bounds: &self.torque_bounds,
                actuator_effort: self
                    .coupled_actuation_enabled
                    .then_some(ActuatorEffortInput {
                        actuation: &self.program.actuation,
                        bounds: &self.actuator_effort_bounds,
                    }),
                contacts: &self.contacts,
                support_patches: &self.support_patches,
            };
            if fixed_actuator_effort.is_some() {
                self.controller
                    .solve_with_fixed_generalized_effort_into(
                        solve_input,
                        &self.mapped_actuator_effort,
                        &mut self.output,
                        &mut self.realization_scratch,
                    )
                    .map_err(value_error)?;
            } else {
                self.controller
                    .solve_into(solve_input, &mut self.output, &mut self.scratch)
                    .map_err(value_error)?;
            }
            let mut witness_squared_error = 0.0;
            for coordinate in 0..generalized_dof {
                generalized_acceleration_out[[tick, coordinate]] =
                    self.output.generalized_acceleration[coordinate];
                let difference = self.output.generalized_acceleration[coordinate]
                    - self.desired_acceleration[coordinate];
                witness_squared_error += difference * difference;
            }
            witness_acceleration_rms_out[tick] =
                (witness_squared_error / generalized_dof as f64).sqrt();
            if self.coupled_actuation_enabled {
                self.program
                    .actuation
                    .write_actuator_effort(
                        &self.output.actuator_torque,
                        &mut self.mapped_actuator_effort,
                    )
                    .map_err(value_error)?;
            }
            let authority_effort = if self.coupled_actuation_enabled {
                &self.mapped_actuator_effort
            } else {
                &self.output.actuator_torque
            };
            for coordinate in 0..dof {
                actuator_torque_out[[tick, coordinate]] = authority_effort[coordinate];
            }
            for contact in 0..self.maximum_contacts {
                contact_normal_force_out[[tick, contact]] =
                    self.output.contact_force_basis[(2, contact)];
                if let Some(forces) = contact_force_basis_out.as_mut() {
                    for axis in 0..3 {
                        forces[[tick, contact, axis]] =
                            self.output.contact_force_basis[(axis, contact)];
                    }
                }
            }
            for slot in 0..FLOATING_TASK_DIAGNOSTIC_CAPACITY {
                task_rms_out[[tick, slot]] = self.output.task_residuals[slot].rms;
                task_clipped_out[[tick, slot]] = u8::from(self.output.task_residuals[slot].clipped);
            }
            dynamics_residual_out[tick] = self.output.dynamics_residual_linf;
            contact_residual_out[tick] = self.output.contact_acceleration_residual_linf;
            minimum_friction_margin_out[tick] = self.output.minimum_friction_margin;
            minimum_support_margin_out[tick] = self.output.minimum_support_margin_m;
            minimum_torque_margin_out[tick] = if self.coupled_actuation_enabled {
                self.output.minimum_actuator_effort_margin
            } else {
                self.output.minimum_torque_margin
            };
            maximum_constraint_violation_out[tick] = self.output.solve.maximum_constraint_violation;
            minimum_bound_margin_out[tick] = self.output.solve.minimum_bound_margin;
            match minimum_joint_position_headroom(&self.program.model, &self.state.robot)
                .map_err(value_error)?
            {
                Some(headroom) => {
                    minimum_joint_margin_rad_out[tick] = headroom.margin_rad;
                    minimum_joint_headroom_fraction_out[tick] = headroom.fraction_of_range;
                    limiting_joint_out[tick] = headroom.coordinate.min(u16::MAX as usize) as u16;
                }
                None => {
                    minimum_joint_margin_rad_out[tick] = f64::NAN;
                    minimum_joint_headroom_fraction_out[tick] = f64::NAN;
                    limiting_joint_out[tick] = u16::MAX;
                }
            }
            match maximum_actuator_effort_utilization(
                authority_effort,
                if self.coupled_actuation_enabled {
                    &self.actuator_effort_bounds
                } else {
                    &self.torque_bounds
                },
            )
            .map_err(value_error)?
            {
                Some(utilization) => {
                    maximum_torque_utilization_out[tick] = utilization.utilization;
                    minimum_torque_headroom_out[tick] = utilization.headroom;
                    limiting_actuator_out[tick] =
                        utilization.coordinate.min(u16::MAX as usize) as u16;
                }
                None => {
                    maximum_torque_utilization_out[tick] = f64::NAN;
                    minimum_torque_headroom_out[tick] = f64::NAN;
                    limiting_actuator_out[tick] = u16::MAX;
                }
            }
            status_out[tick] = match self.output.status {
                SolveStatus::Solved => 0,
                SolveStatus::SolvedWithSlack => 1,
                SolveStatus::PrimalInfeasible => 2,
                SolveStatus::NumericalFailure | SolveStatus::InvalidProblem => 3,
                SolveStatus::MaxIterations => 4,
            };
            task_pseudoinverse_calls_out[tick] = self
                .output
                .solve
                .task_pseudoinverse_calls
                .min(u16::MAX as usize) as u16;
            for priority in 0..Priority::ALL.len() {
                task_pseudoinverse_calls_by_priority_out[[tick, priority]] =
                    self.output.solve.task_pseudoinverse_calls_by_level[priority]
                        .min(u16::MAX as usize) as u16;
            }
            clipped_steps_out[tick] = self.output.solve.clipped_steps.min(u16::MAX as usize) as u16;
            for priority in 0..Priority::ALL.len() {
                clipped_steps_by_priority_out[[tick, priority]] =
                    self.output.solve.clipped_steps_by_level[priority].min(u16::MAX as usize)
                        as u16;
            }
            task_jacobi_sweeps_out[tick] =
                self.output.solve.task_jacobi_sweeps.min(u16::MAX as usize) as u16;
            for priority in 0..Priority::ALL.len() {
                task_jacobi_sweeps_by_priority_out[[tick, priority]] =
                    self.output.solve.task_jacobi_sweeps_by_level[priority].min(u16::MAX as usize)
                        as u16;
            }
            feasibility_projection_sweeps_out[tick] =
                self.output
                    .solve
                    .feasibility_projection_sweeps
                    .min(u16::MAX as usize) as u16;
            feasibility_halfspace_projections_out[tick] =
                self.output
                    .solve
                    .feasibility_halfspace_projections
                    .min(u32::MAX as usize) as u32;
            feasibility_polish_iterations_out[tick] =
                self.output
                    .solve
                    .feasibility_polish_iterations
                    .min(u16::MAX as usize) as u16;
            if let Some(values) = feasibility_seed_reused_out.as_deref_mut() {
                values[tick] = self.output.solve.feasibility_seed_reused as u8;
            }
            if let Some(values) = feasibility_prefix_resumed_out.as_deref_mut() {
                values[tick] = self.output.solve.feasibility_prefix_resumed as u8;
            }
            step_ns_out[tick] = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let (allocation_calls_after, allocated_bytes_after) = allocation_snapshot();
            allocation_calls_out[tick] =
                allocation_calls_after.saturating_sub(allocation_calls_before);
            allocated_bytes_out[tick] =
                allocated_bytes_after.saturating_sub(allocated_bytes_before);
        }
        self.acceleration_bounds
            .lower
            .copy_from(&self.nominal_acceleration_bounds.lower);
        self.acceleration_bounds
            .upper
            .copy_from(&self.nominal_acceleration_bounds.upper);
        self.torque_bounds
            .lower
            .copy_from(&self.nominal_torque_bounds.lower);
        self.torque_bounds
            .upper
            .copy_from(&self.nominal_torque_bounds.upper);
        self.actuator_effort_bounds
            .lower
            .copy_from(&self.nominal_actuator_effort_bounds.lower);
        self.actuator_effort_bounds
            .upper
            .copy_from(&self.nominal_actuator_effort_bounds.upper);
        Ok(())
    }

    /// Run a moving-root floating WBC trace entirely inside Rust.
    ///
    /// Point rows and contact rows share the same target jets. A stance point
    /// is a hard three-axis acceleration lock at its measured touchdown
    /// anchor; swing points occupy the fixed soft Cartesian task slots.
    #[allow(clippy::too_many_arguments)]
    fn run_trace(
        &mut self,
        dt_seconds: f64,
        root_target_positions: PyReadonlyArray2<'_, f64>,
        root_target_velocities: PyReadonlyArray2<'_, f64>,
        root_target_accelerations: PyReadonlyArray2<'_, f64>,
        center_of_mass_target_positions: PyReadonlyArray2<'_, f64>,
        center_of_mass_target_velocities: PyReadonlyArray2<'_, f64>,
        center_of_mass_target_accelerations: PyReadonlyArray2<'_, f64>,
        frame_ids: PyReadonlyArray1<'_, i64>,
        target_positions: PyReadonlyArray3<'_, f64>,
        target_velocities: PyReadonlyArray3<'_, f64>,
        target_accelerations: PyReadonlyArray3<'_, f64>,
        target_active: PyReadonlyArray2<'_, u8>,
        contact_active: PyReadonlyArray2<'_, u8>,
        priorities: PyReadonlyArray1<'_, u8>,
        weights: PyReadonlyArray1<'_, f64>,
        posture: PyReadonlyArray1<'_, f64>,
        posture_positions: PyReadonlyArray2<'_, f64>,
        posture_velocities: PyReadonlyArray2<'_, f64>,
        posture_accelerations: PyReadonlyArray2<'_, f64>,
        protected_joint_coordinates: PyReadonlyArray1<'_, i64>,
        mut root_translation_out: PyReadwriteArray2<'_, f64>,
        mut root_quaternion_wxyz_out: PyReadwriteArray2<'_, f64>,
        mut center_of_mass_tracked_out: PyReadwriteArray2<'_, f64>,
        mut q_out: PyReadwriteArray2<'_, f64>,
        mut v_out: PyReadwriteArray2<'_, f64>,
        mut tracked_positions_out: PyReadwriteArray3<'_, f64>,
        mut contact_normal_force_out: PyReadwriteArray2<'_, f64>,
        mut task_rms_out: PyReadwriteArray2<'_, f64>,
        mut dynamics_residual_out: PyReadwriteArray1<'_, f64>,
        mut contact_residual_out: PyReadwriteArray1<'_, f64>,
        mut minimum_support_margin_out: PyReadwriteArray1<'_, f64>,
        mut step_ns_out: PyReadwriteArray1<'_, u64>,
        mut status_out: PyReadwriteArray1<'_, u8>,
        mut support_phase_out: PyReadwriteArray2<'_, u8>,
        mut support_phase_ticks_out: PyReadwriteArray2<'_, u16>,
        mut support_tangential_speed_out: PyReadwriteArray2<'_, f64>,
        mut support_touchdown_position_error_out: PyReadwriteArray2<'_, f64>,
        mut support_touchdown_normal_speed_out: PyReadwriteArray2<'_, f64>,
        mut task_pseudoinverse_calls_out: PyReadwriteArray1<'_, u16>,
        mut clipped_steps_out: PyReadwriteArray1<'_, u16>,
        mut task_pseudoinverse_calls_by_level_out: PyReadwriteArray2<'_, u16>,
        mut clipped_steps_by_level_out: PyReadwriteArray2<'_, u16>,
        mut task_jacobi_sweeps_out: PyReadwriteArray1<'_, u16>,
        mut task_jacobi_sweeps_by_level_out: PyReadwriteArray2<'_, u16>,
        mut feasibility_projection_sweeps_out: PyReadwriteArray1<'_, u16>,
        mut feasibility_halfspace_projections_out: PyReadwriteArray1<'_, u32>,
        mut feasibility_polish_iterations_out: PyReadwriteArray1<'_, u16>,
        mut feasibility_polish_pseudoinverse_calls_out: PyReadwriteArray1<'_, u16>,
        mut feasibility_polish_jacobi_sweeps_out: PyReadwriteArray1<'_, u16>,
        mut center_of_mass_velocity_out: PyReadwriteArray2<'_, f64>,
        mut dcm_out: PyReadwriteArray2<'_, f64>,
        mut target_dcm_out: PyReadwriteArray2<'_, f64>,
        mut virtual_zmp_out: PyReadwriteArray2<'_, f64>,
        mut clipped_zmp_out: PyReadwriteArray2<'_, f64>,
        mut center_of_mass_command_out: PyReadwriteArray2<'_, f64>,
        mut dcm_natural_frequency_out: PyReadwriteArray1<'_, f64>,
        mut dcm_measured_height_out: PyReadwriteArray1<'_, f64>,
        mut dcm_height_clamped_out: PyReadwriteArray1<'_, u8>,
        mut dcm_zmp_clipped_out: PyReadwriteArray1<'_, u8>,
        mut dcm_support_vertices_out: PyReadwriteArray1<'_, u8>,
        mut dcm_support_margin_out: PyReadwriteArray1<'_, f64>,
        mut landing_retarget_anchor_out: PyReadwriteArray3<'_, f64>,
        mut landing_retarget_capture_scale_out: PyReadwriteArray2<'_, f64>,
        mut landing_retarget_offset_out: PyReadwriteArray2<'_, f64>,
        mut landing_retarget_reach_out: PyReadwriteArray2<'_, f64>,
        mut landing_retarget_flags_out: PyReadwriteArray2<'_, u8>,
        mut contact_phase_authority_out: PyReadwriteArray1<'_, u8>,
        mut joint_velocity_envelope_target_scale_out: PyReadwriteArray1<'_, f64>,
        mut joint_velocity_envelope_scale_out: PyReadwriteArray1<'_, f64>,
        mut joint_velocity_envelope_active_coordinates_out: PyReadwriteArray1<'_, u8>,
        mut effective_root_target_out: PyReadwriteArray2<'_, f64>,
        mut effective_center_of_mass_target_out: PyReadwriteArray2<'_, f64>,
        mut effective_target_positions_out: PyReadwriteArray3<'_, f64>,
        mut effective_contact_active_out: PyReadwriteArray2<'_, u8>,
        mut reference_phase_out: PyReadwriteArray1<'_, f64>,
        mut reference_phase_target_rate_out: PyReadwriteArray1<'_, f64>,
        mut reference_phase_rate_out: PyReadwriteArray1<'_, f64>,
        mut reference_phase_acceleration_out: PyReadwriteArray1<'_, f64>,
        mut reference_phase_required_time_out: PyReadwriteArray1<'_, f64>,
        mut reference_phase_flags_out: PyReadwriteArray1<'_, u8>,
    ) -> PyResult<()> {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return Err(PyValueError::new_err("dt_seconds must be positive"));
        }
        let root_target_positions = root_target_positions.as_array();
        let root_target_velocities = root_target_velocities.as_array();
        let root_target_accelerations = root_target_accelerations.as_array();
        let center_of_mass_target_positions = center_of_mass_target_positions.as_array();
        let center_of_mass_target_velocities = center_of_mass_target_velocities.as_array();
        let center_of_mass_target_accelerations = center_of_mass_target_accelerations.as_array();
        let frame_ids = frame_ids.as_slice()?;
        let target_positions = target_positions.as_array();
        let target_velocities = target_velocities.as_array();
        let target_accelerations = target_accelerations.as_array();
        let target_active = target_active.as_array();
        let contact_active = contact_active.as_array();
        let priorities = priorities.as_slice()?;
        let weights = weights.as_slice()?;
        let posture = posture.as_slice()?;
        let posture_positions = posture_positions.as_array();
        let posture_velocities = posture_velocities.as_array();
        let posture_accelerations = posture_accelerations.as_array();
        let protected_joint_coordinates = protected_joint_coordinates.as_slice()?;
        let mut root_translation_out = root_translation_out.as_array_mut();
        let mut root_quaternion_wxyz_out = root_quaternion_wxyz_out.as_array_mut();
        let mut center_of_mass_tracked_out = center_of_mass_tracked_out.as_array_mut();
        let mut q_out = q_out.as_array_mut();
        let mut v_out = v_out.as_array_mut();
        let mut tracked_positions_out = tracked_positions_out.as_array_mut();
        let mut contact_normal_force_out = contact_normal_force_out.as_array_mut();
        let mut task_rms_out = task_rms_out.as_array_mut();
        let dynamics_residual_out = dynamics_residual_out.as_slice_mut()?;
        let contact_residual_out = contact_residual_out.as_slice_mut()?;
        let minimum_support_margin_out = minimum_support_margin_out.as_slice_mut()?;
        let step_ns_out = step_ns_out.as_slice_mut()?;
        let status_out = status_out.as_slice_mut()?;
        let mut support_phase_out = support_phase_out.as_array_mut();
        let mut support_phase_ticks_out = support_phase_ticks_out.as_array_mut();
        let mut support_tangential_speed_out = support_tangential_speed_out.as_array_mut();
        let mut support_touchdown_position_error_out =
            support_touchdown_position_error_out.as_array_mut();
        let mut support_touchdown_normal_speed_out =
            support_touchdown_normal_speed_out.as_array_mut();
        let task_pseudoinverse_calls_out = task_pseudoinverse_calls_out.as_slice_mut()?;
        let clipped_steps_out = clipped_steps_out.as_slice_mut()?;
        let mut task_pseudoinverse_calls_by_level_out =
            task_pseudoinverse_calls_by_level_out.as_array_mut();
        let mut clipped_steps_by_level_out = clipped_steps_by_level_out.as_array_mut();
        let task_jacobi_sweeps_out = task_jacobi_sweeps_out.as_slice_mut()?;
        let mut task_jacobi_sweeps_by_level_out = task_jacobi_sweeps_by_level_out.as_array_mut();
        let feasibility_projection_sweeps_out = feasibility_projection_sweeps_out.as_slice_mut()?;
        let feasibility_halfspace_projections_out =
            feasibility_halfspace_projections_out.as_slice_mut()?;
        let feasibility_polish_iterations_out = feasibility_polish_iterations_out.as_slice_mut()?;
        let feasibility_polish_pseudoinverse_calls_out =
            feasibility_polish_pseudoinverse_calls_out.as_slice_mut()?;
        let feasibility_polish_jacobi_sweeps_out =
            feasibility_polish_jacobi_sweeps_out.as_slice_mut()?;
        let mut center_of_mass_velocity_out = center_of_mass_velocity_out.as_array_mut();
        let mut dcm_out = dcm_out.as_array_mut();
        let mut target_dcm_out = target_dcm_out.as_array_mut();
        let mut virtual_zmp_out = virtual_zmp_out.as_array_mut();
        let mut clipped_zmp_out = clipped_zmp_out.as_array_mut();
        let mut center_of_mass_command_out = center_of_mass_command_out.as_array_mut();
        let dcm_natural_frequency_out = dcm_natural_frequency_out.as_slice_mut()?;
        let dcm_measured_height_out = dcm_measured_height_out.as_slice_mut()?;
        let dcm_height_clamped_out = dcm_height_clamped_out.as_slice_mut()?;
        let dcm_zmp_clipped_out = dcm_zmp_clipped_out.as_slice_mut()?;
        let dcm_support_vertices_out = dcm_support_vertices_out.as_slice_mut()?;
        let dcm_support_margin_out = dcm_support_margin_out.as_slice_mut()?;
        let mut landing_retarget_anchor_out = landing_retarget_anchor_out.as_array_mut();
        let mut landing_retarget_capture_scale_out =
            landing_retarget_capture_scale_out.as_array_mut();
        let mut landing_retarget_offset_out = landing_retarget_offset_out.as_array_mut();
        let mut landing_retarget_reach_out = landing_retarget_reach_out.as_array_mut();
        let mut landing_retarget_flags_out = landing_retarget_flags_out.as_array_mut();
        let contact_phase_authority_out = contact_phase_authority_out.as_slice_mut()?;
        let joint_velocity_envelope_target_scale_out =
            joint_velocity_envelope_target_scale_out.as_slice_mut()?;
        let joint_velocity_envelope_scale_out = joint_velocity_envelope_scale_out.as_slice_mut()?;
        let joint_velocity_envelope_active_coordinates_out =
            joint_velocity_envelope_active_coordinates_out.as_slice_mut()?;
        let mut effective_root_target_out = effective_root_target_out.as_array_mut();
        let mut effective_center_of_mass_target_out =
            effective_center_of_mass_target_out.as_array_mut();
        let mut effective_target_positions_out = effective_target_positions_out.as_array_mut();
        let mut effective_contact_active_out = effective_contact_active_out.as_array_mut();
        let reference_phase_out = reference_phase_out.as_slice_mut()?;
        let reference_phase_target_rate_out = reference_phase_target_rate_out.as_slice_mut()?;
        let reference_phase_rate_out = reference_phase_rate_out.as_slice_mut()?;
        let reference_phase_acceleration_out = reference_phase_acceleration_out.as_slice_mut()?;
        let reference_phase_required_time_out = reference_phase_required_time_out.as_slice_mut()?;
        let reference_phase_flags_out = reference_phase_flags_out.as_slice_mut()?;

        let ticks = root_target_positions.shape()[0];
        let target_count = frame_ids.len();
        let dof = self.dof();
        let generalized_dof = dof + 6;
        let valid_shapes = target_count <= FLOATING_POINT_TASK_CAPACITY
            && root_target_positions.shape() == [ticks, 3]
            && root_target_velocities.shape() == [ticks, 3]
            && root_target_accelerations.shape() == [ticks, 3]
            && center_of_mass_target_positions.shape() == [ticks, 3]
            && center_of_mass_target_velocities.shape() == [ticks, 3]
            && center_of_mass_target_accelerations.shape() == [ticks, 3]
            && target_positions.shape() == [ticks, target_count, 3]
            && target_velocities.shape() == [ticks, target_count, 3]
            && target_accelerations.shape() == [ticks, target_count, 3]
            && target_active.shape() == [ticks, target_count]
            && contact_active.shape() == [ticks, target_count]
            && priorities.len() == target_count
            && weights.len() == target_count
            && posture.len() == dof
            && posture_positions.shape() == [ticks, dof]
            && posture_velocities.shape() == [ticks, dof]
            && posture_accelerations.shape() == [ticks, dof]
            && root_translation_out.shape() == [ticks, 3]
            && root_quaternion_wxyz_out.shape() == [ticks, 4]
            && center_of_mass_tracked_out.shape() == [ticks, 3]
            && q_out.shape() == [ticks, dof]
            && v_out.shape() == [ticks, dof]
            && tracked_positions_out.shape() == [ticks, target_count, 3]
            && contact_normal_force_out.shape() == [ticks, self.maximum_contacts]
            && task_rms_out.shape() == [ticks, FLOATING_TASK_DIAGNOSTIC_CAPACITY]
            && dynamics_residual_out.len() == ticks
            && contact_residual_out.len() == ticks
            && minimum_support_margin_out.len() == ticks
            && step_ns_out.len() == ticks
            && status_out.len() == ticks
            && support_phase_out.shape() == [ticks, target_count]
            && support_phase_ticks_out.shape() == [ticks, target_count]
            && support_tangential_speed_out.shape() == [ticks, target_count]
            && support_touchdown_position_error_out.shape() == [ticks, target_count]
            && support_touchdown_normal_speed_out.shape() == [ticks, target_count]
            && task_pseudoinverse_calls_out.len() == ticks
            && clipped_steps_out.len() == ticks
            && task_pseudoinverse_calls_by_level_out.shape() == [ticks, Priority::ALL.len()]
            && clipped_steps_by_level_out.shape() == [ticks, Priority::ALL.len()]
            && task_jacobi_sweeps_out.len() == ticks
            && task_jacobi_sweeps_by_level_out.shape() == [ticks, Priority::ALL.len()]
            && feasibility_projection_sweeps_out.len() == ticks
            && feasibility_halfspace_projections_out.len() == ticks
            && feasibility_polish_iterations_out.len() == ticks
            && feasibility_polish_pseudoinverse_calls_out.len() == ticks
            && feasibility_polish_jacobi_sweeps_out.len() == ticks
            && center_of_mass_velocity_out.shape() == [ticks, 3]
            && dcm_out.shape() == [ticks, 3]
            && target_dcm_out.shape() == [ticks, 3]
            && virtual_zmp_out.shape() == [ticks, 3]
            && clipped_zmp_out.shape() == [ticks, 3]
            && center_of_mass_command_out.shape() == [ticks, 3]
            && dcm_natural_frequency_out.len() == ticks
            && dcm_measured_height_out.len() == ticks
            && dcm_height_clamped_out.len() == ticks
            && dcm_zmp_clipped_out.len() == ticks
            && dcm_support_vertices_out.len() == ticks
            && dcm_support_margin_out.len() == ticks
            && landing_retarget_anchor_out.shape() == [ticks, target_count, 3]
            && landing_retarget_capture_scale_out.shape() == [ticks, target_count]
            && landing_retarget_offset_out.shape() == [ticks, target_count]
            && landing_retarget_reach_out.shape() == [ticks, target_count]
            && landing_retarget_flags_out.shape() == [ticks, target_count]
            && contact_phase_authority_out.len() == ticks
            && joint_velocity_envelope_target_scale_out.len() == ticks
            && joint_velocity_envelope_scale_out.len() == ticks
            && joint_velocity_envelope_active_coordinates_out.len() == ticks
            && effective_root_target_out.shape() == [ticks, 3]
            && effective_center_of_mass_target_out.shape() == [ticks, 3]
            && effective_target_positions_out.shape() == [ticks, target_count, 3]
            && effective_contact_active_out.shape() == [ticks, target_count]
            && reference_phase_out.len() == ticks
            && reference_phase_target_rate_out.len() == ticks
            && reference_phase_rate_out.len() == ticks
            && reference_phase_acceleration_out.len() == ticks
            && reference_phase_required_time_out.len() == ticks
            && reference_phase_flags_out.len() == ticks;
        if !valid_shapes {
            return Err(PyValueError::new_err(
                "floating run_trace array shape mismatch",
            ));
        }
        if frame_ids
            .iter()
            .any(|&frame| frame < 0 || frame as usize >= self.program.model.bodies.len())
        {
            return Err(PyValueError::new_err(
                "floating run_trace frame id out of range",
            ));
        }
        self.protected_joint_coordinates.clear();
        for &coordinate in protected_joint_coordinates {
            if coordinate < 0 || coordinate as usize >= dof {
                return Err(PyValueError::new_err(
                    "protected joint coordinate is out of range",
                ));
            }
            let coordinate = coordinate as usize;
            if self.protected_joint_coordinates.last().copied() >= Some(coordinate) {
                return Err(PyValueError::new_err(
                    "protected joint coordinates must be strictly increasing",
                ));
            }
            self.protected_joint_coordinates.push(coordinate);
        }
        let point_priorities = priorities
            .iter()
            .copied()
            .map(priority)
            .collect::<PyResult<Vec<_>>>()?;
        if weights
            .iter()
            .any(|weight| !weight.is_finite() || *weight < 0.0)
        {
            return Err(PyValueError::new_err(
                "floating run_trace weights must be finite and nonnegative",
            ));
        }

        let contact_omega = std::f64::consts::TAU;
        let posture_omega = std::f64::consts::TAU;
        for tick in 0..ticks {
            let started = Instant::now();
            let reference_phase = if self.reference_phase_retiming_enabled {
                self.reference_phase
            } else {
                tick as f64
            };
            let reference_tick = reference_phase.floor() as usize;
            let reference_next_tick = (reference_tick + 1).min(ticks.saturating_sub(1));
            let reference_fraction = reference_phase - reference_tick as f64;
            // A newly requested target whose own release latch is clear is an
            // explicit recovery edge for the session-level free-body
            // fallback. Failed targets remain suppressed independently, but
            // one stale authored request must not prevent a different target
            // from reacquiring after its genuine swing/re-touchdown edge.
            if (0..target_count).any(|target| {
                contact_active[[reference_tick, target]] != 0
                    && target_active[[reference_tick, target]] != 0
                    && !self.contact_release_suppressed[target]
            }) {
                self.no_contact_safe_mode = false;
            }
            let phase_rate = if self.reference_phase_retiming_enabled {
                self.reference_phase_rate
            } else {
                1.0
            };
            let phase_acceleration = if self.reference_phase_retiming_enabled {
                (self.reference_phase_rate - self.previous_reference_phase_rate) / dt_seconds
            } else {
                0.0
            };
            reference_phase_out[tick] = reference_phase;
            reference_phase_rate_out[tick] = phase_rate;
            reference_phase_acceleration_out[tick] = phase_acceleration;
            let mut phase_target_rate = 1.0_f64;
            let mut phase_required_time_seconds = 0.0_f64;
            let mut phase_flags = 0_u8;
            self.program
                .model
                .forward_kinematics(&self.state.robot, &mut self.tracking_cache)
                .map_err(value_error)?;
            self.generalized_velocity.as_mut_slice()[..6]
                .copy_from_slice(self.state.root_twist_world.0.as_slice());
            self.generalized_velocity.as_mut_slice()[6..]
                .copy_from_slice(self.state.robot.v.as_slice());
            for coordinate in 0..generalized_dof {
                let maximum_velocity = if coordinate < 3 {
                    8.0
                } else if coordinate < 6 {
                    5.0
                } else {
                    8.0
                };
                self.acceleration_bounds.lower[coordinate] = (-self.maximum_acceleration)
                    .max((-maximum_velocity - self.generalized_velocity[coordinate]) / dt_seconds);
                self.acceleration_bounds.upper[coordinate] = self
                    .maximum_acceleration
                    .min((maximum_velocity - self.generalized_velocity[coordinate]) / dt_seconds);
            }
            if self.joint_limit_braking {
                for joint in &self.program.model.joints {
                    let Some(coordinate) = joint.coordinate else {
                        continue;
                    };
                    let generalized_coordinate = 6 + coordinate;
                    let maximum_velocity = joint.limit.velocity.abs().min(8.0);
                    let (lower, upper) = joint_acceleration_interval(
                        self.state.robot.q[coordinate],
                        self.state.robot.v[coordinate],
                        joint.limit.lower,
                        joint.limit.upper,
                        maximum_velocity,
                        self.maximum_acceleration,
                        dt_seconds,
                    )
                    .ok_or_else(|| {
                        PyValueError::new_err("authored joint acceleration limits are invalid")
                    })?;
                    self.acceleration_bounds.lower[generalized_coordinate] = lower;
                    self.acceleration_bounds.upper[generalized_coordinate] = upper;
                }
            }

            self.desired_acceleration.fill(0.0);
            let identity = UnitQuaternion::identity();
            let root_rotation = self.state.robot.control_world_from_root.rotation;
            let local_rotation_error = root_rotation.rotation_to(&identity).scaled_axis();
            let rotation_error_world = root_rotation.transform_vector(&local_rotation_error);
            let angular_velocity =
                Vec3::from_row_slice(&self.state.root_twist_world.0.as_slice()[..3]);
            let angular_acceleration = clamp_norm(
                self.root_omega * self.root_omega * rotation_error_world
                    - 2.0 * self.root_omega * angular_velocity,
                100.0,
            );
            self.desired_acceleration.as_mut_slice()[..3]
                .copy_from_slice(angular_acceleration.as_slice());
            let root_target = sample_array2_vector_jet(
                &root_target_positions,
                &root_target_velocities,
                &root_target_accelerations,
                reference_tick,
                reference_next_tick,
                reference_fraction,
                dt_seconds,
                phase_rate,
                phase_acceleration,
            )?;
            let root_target_position = root_target.value;
            let root_target_velocity = root_target.velocity;
            let root_target_acceleration = root_target.acceleration;
            for axis in 0..3 {
                effective_root_target_out[[tick, axis]] = root_target_position[axis];
            }
            let root_position = self.state.robot.control_world_from_root.translation.vector;
            let root_velocity =
                Vec3::from_row_slice(&self.state.root_twist_world.0.as_slice()[3..6]);
            let desired_root_linear = tracking_acceleration(
                root_target_position,
                root_target_velocity,
                root_target_acceleration,
                root_position,
                root_velocity,
                self.root_omega,
                100.0,
            );
            let center_of_mass_world = self.tracking_cache.center_of_mass_world;
            for axis in 0..3 {
                center_of_mass_tracked_out[[tick, axis]] = center_of_mass_world[axis];
            }
            let center_of_mass_velocity_world =
                (center_of_mass_world - self.previous_center_of_mass_world) / dt_seconds;
            self.previous_center_of_mass_world = center_of_mass_world;
            for axis in 0..3 {
                center_of_mass_velocity_out[[tick, axis]] = center_of_mass_velocity_world[axis];
            }
            let center_of_mass_target = sample_array2_vector_jet(
                &center_of_mass_target_positions,
                &center_of_mass_target_velocities,
                &center_of_mass_target_accelerations,
                reference_tick,
                reference_next_tick,
                reference_fraction,
                dt_seconds,
                phase_rate,
                phase_acceleration,
            )?;
            let center_of_mass_target_position = center_of_mass_target.value;
            let center_of_mass_target_velocity = center_of_mass_target.velocity;
            let center_of_mass_target_acceleration = center_of_mass_target.acceleration;
            for axis in 0..3 {
                effective_center_of_mass_target_out[[tick, axis]] =
                    center_of_mass_target_position[axis];
            }
            let desired_center_of_mass_tracking = tracking_acceleration(
                center_of_mass_target_position,
                center_of_mass_target_velocity,
                center_of_mass_target_acceleration,
                center_of_mass_world,
                center_of_mass_velocity_world,
                self.center_of_mass_omega,
                100.0,
            );
            let centroidal_angular_momentum_task = if self.centroidal_angular_momentum_weight > 0.0
            {
                self.program
                    .model
                    .floating_centroidal_map_into(
                        &self.tracking_cache,
                        &mut self.centroidal_dynamics,
                        &mut self.centroidal_map,
                    )
                    .map_err(value_error)?;
                let angular_momentum_world = Vec3::from_fn(|axis, _| {
                    (0..generalized_dof)
                        .map(|coordinate| {
                            self.centroidal_map[(axis, coordinate)]
                                * self.generalized_velocity[coordinate]
                        })
                        .sum()
                });
                Some(FloatingCentroidalAngularMomentumTask {
                    desired_rate_world: -self.centroidal_angular_momentum_omega
                        * angular_momentum_world,
                    priority: self.centroidal_angular_momentum_priority,
                    weight: self.centroidal_angular_momentum_weight,
                })
            } else {
                None
            };
            self.desired_acceleration.as_mut_slice()[3..6]
                .copy_from_slice(desired_root_linear.as_slice());
            for (coordinate, &fallback_posture_target) in posture.iter().enumerate() {
                let posture_target = sample_array2_scalar_jet(
                    &posture_positions,
                    &posture_velocities,
                    &posture_accelerations,
                    coordinate,
                    reference_tick,
                    reference_next_tick,
                    reference_fraction,
                    dt_seconds,
                    phase_rate,
                    phase_acceleration,
                )?;
                let target_position = if posture_target.value.is_finite() {
                    posture_target.value
                } else {
                    fallback_posture_target
                };
                self.desired_acceleration[6 + coordinate] = (posture_target.acceleration
                    + posture_omega
                        * posture_omega
                        * (target_position - self.state.robot.q[coordinate])
                    + 2.0
                        * posture_omega
                        * (posture_target.velocity - self.state.robot.v[coordinate]))
                    .clamp(-100.0, 100.0);
            }
            for (slot, &coordinate) in self.protected_joint_coordinates.iter().enumerate() {
                self.protected_joint_accelerations[slot] =
                    self.desired_acceleration[6 + coordinate];
            }
            let protected_joint_task = (!self.protected_joint_coordinates.is_empty()
                && self.protected_joint_posture_weight > 0.0)
                .then_some(FloatingJointAccelerationTask {
                    coordinates: &self.protected_joint_coordinates,
                    desired_accelerations: &self.protected_joint_accelerations
                        [..self.protected_joint_coordinates.len()],
                    priority: self.protected_joint_posture_priority,
                    weight: self.protected_joint_posture_weight,
                });
            let established_support_count = self
                .support_transitions
                .iter()
                .filter(|state| state.phase.is_contact())
                .count();
            let has_precontact_target = self
                .support_transitions
                .iter()
                .any(|state| state.phase == SupportPhase::Precontact);
            let contact_phase_authority = contact_phase_authority(
                established_support_count,
                has_precontact_target,
                self.contact_phase_authority_config,
            )
            .ok_or_else(|| PyValueError::new_err("contact-phase authority config is invalid"))?;
            contact_phase_authority_out[tick] = contact_phase_authority.phase as u8;

            self.point_tasks.clear();
            self.angular_tasks.clear();
            self.contacts.clear();
            self.support_patches.clear();
            for target in 0..target_count {
                support_tangential_speed_out[[tick, target]] = f64::NAN;
                support_touchdown_position_error_out[[tick, target]] = f64::NAN;
                support_touchdown_normal_speed_out[[tick, target]] = f64::NAN;
                for axis in 0..3 {
                    landing_retarget_anchor_out[[tick, target, axis]] = f64::NAN;
                }
                landing_retarget_capture_scale_out[[tick, target]] = f64::NAN;
                landing_retarget_offset_out[[tick, target]] = f64::NAN;
                landing_retarget_reach_out[[tick, target]] = f64::NAN;
                landing_retarget_flags_out[[tick, target]] = 0;
            }
            let mut precontact_transition = false;
            // A scheduled handoff may ask the old support to release before
            // the incoming material point has actually landed. Keep that
            // support until the requested replacement is locked; the motion
            // schedule is intent, measured contact state is authority.
            let has_pending_requested_support = (0..target_count).any(|target| {
                contact_active[[reference_tick, target]] != 0
                    && target_active[[reference_tick, target]] != 0
                    && !self.contact_release_suppressed[target]
                    && !matches!(
                        self.support_transitions[target].phase,
                        SupportPhase::Locked | SupportPhase::NormalFallback
                    )
            });
            let has_established_requested_support = (0..target_count).any(|target| {
                contact_active[[reference_tick, target]] != 0
                    && target_active[[reference_tick, target]] != 0
                    && !self.contact_release_suppressed[target]
                    && matches!(
                        self.support_transitions[target].phase,
                        SupportPhase::Locked | SupportPhase::NormalFallback
                    )
            });
            for target in 0..target_count {
                if contact_active[[reference_tick, target]] != 0
                    && target_active[[reference_tick, target]] == 0
                {
                    return Err(PyValueError::new_err(
                        "contact-active target must also be target-active",
                    ));
                }
                if target_active[[reference_tick, target]] == 0 {
                    self.support_transitions[target].clear();
                    self.contact_release_suppressed[target] = false;
                    self.precontact_planned[target] = false;
                    self.precontact_authored_anchor_world[target] = Vec3::zeros();
                    self.precontact_anchor_world[target] = Vec3::zeros();
                    self.precontact_future_root_world[target] = Vec3::zeros();
                    self.precontact_rotation_world[target] = UnitQuaternion::identity();
                    self.contact_anchor_world[target] = Vec3::zeros();
                    self.contact_patch_anchor_world[target] = [Vec3::zeros(); 4];
                    continue;
                }
                let frame = bonesaw_core::FrameId(frame_ids[target] as usize);
                self.program
                    .model
                    .floating_point_jacobian_into(
                        &self.tracking_cache,
                        frame,
                        Vec3::zeros(),
                        &mut self.tracking_jacobian,
                    )
                    .map_err(value_error)?;
                let current_position = self.tracking_cache.world_from_body[frame.0]
                    .translation
                    .vector;
                let current_velocity = Vec3::from_fn(|axis, _| {
                    (0..self.generalized_velocity.len())
                        .map(|coordinate| {
                            self.tracking_jacobian[(axis, coordinate)]
                                * self.generalized_velocity[coordinate]
                        })
                        .sum()
                });
                let target_jet = sample_array3_vector_jet(
                    &target_positions,
                    &target_velocities,
                    &target_accelerations,
                    target,
                    reference_tick,
                    reference_next_tick,
                    reference_fraction,
                    dt_seconds,
                    phase_rate,
                    phase_acceleration,
                )?;
                let target_position = target_jet.value;
                let target_velocity = target_jet.velocity;
                let target_acceleration = target_jet.acceleration;
                for axis in 0..3 {
                    effective_target_positions_out[[tick, target, axis]] = target_position[axis];
                }
                let contact_requested = contact_active[[reference_tick, target]] != 0;
                // A release contingency is sticky while the schedule still
                // requests this contact.  Once the schedule goes through a
                // swing sample, clear the latch so a later touchdown can
                // acquire the contact again.
                if !contact_requested {
                    self.contact_release_suppressed[target] = false;
                }
                let contact_release_suppressed = self.contact_release_suppressed[target];
                effective_contact_active_out[[tick, target]] = u8::from(contact_requested);
                let prior_phase = self.support_transitions[target].phase;
                let precontact_offset = (!contact_requested && self.precontact_ticks > 0)
                    .then(|| {
                        (1..=self.precontact_ticks.min(ticks - reference_tick - 1))
                            .find(|offset| contact_active[[reference_tick + offset, target]] != 0)
                    })
                    .flatten();
                let admission_pending = contact_requested
                    && self.precontact_planned[target]
                    && !prior_phase.is_contact();
                let scheduled_precontact = precontact_offset.is_some();
                // Contact viability and shaping use the same prospective
                // material point on the sole, never the ankle origin.
                let transition_point = if self.contact_points_per_target == 4 {
                    (self.contact_patch_points[0] + self.contact_patch_points[3]) * 0.5
                } else {
                    self.contact_patch_points[0]
                };
                let material_state_required = scheduled_precontact
                    || admission_pending
                    || prior_phase == SupportPhase::TouchdownNormal;
                let (transition_position, transition_velocity) = if material_state_required {
                    self.program
                        .model
                        .floating_point_jacobian_into(
                            &self.tracking_cache,
                            frame,
                            transition_point,
                            &mut self.tracking_jacobian,
                        )
                        .map_err(value_error)?;
                    let transition_velocity = Vec3::from_fn(|axis, _| {
                        (0..self.generalized_velocity.len())
                            .map(|coordinate| {
                                self.tracking_jacobian[(axis, coordinate)]
                                    * self.generalized_velocity[coordinate]
                            })
                            .sum()
                    });
                    let transition_position = self.tracking_cache.world_from_body[frame.0]
                        .transform_point(&Point3::from(transition_point))
                        .coords;
                    (transition_position, transition_velocity)
                } else {
                    (current_position, current_velocity)
                };
                let mut maximum_patch_tangential_speed =
                    transition_velocity.fixed_rows::<2>(0).norm();
                if material_state_required && self.contact_points_per_target == 4 {
                    for point in self.contact_patch_points {
                        self.program
                            .model
                            .floating_point_jacobian_into(
                                &self.tracking_cache,
                                frame,
                                point,
                                &mut self.tracking_jacobian,
                            )
                            .map_err(value_error)?;
                        let point_velocity = Vec3::from_fn(|axis, _| {
                            (0..self.generalized_velocity.len())
                                .map(|coordinate| {
                                    self.tracking_jacobian[(axis, coordinate)]
                                        * self.generalized_velocity[coordinate]
                                })
                                .sum()
                        });
                        maximum_patch_tangential_speed = maximum_patch_tangential_speed
                            .max(point_velocity.fixed_rows::<2>(0).norm());
                    }
                }
                let prospective_precontact_anchor = precontact_offset.map(|offset| {
                    let touchdown_frame_position = Vec3::new(
                        target_positions[[reference_tick + offset, target, 0]],
                        target_positions[[reference_tick + offset, target, 1]],
                        target_positions[[reference_tick + offset, target, 2]],
                    );
                    touchdown_frame_position + (transition_position - current_position)
                });
                let mut target_phase_required_time_seconds = 0.0_f64;
                if self.touchdown_phase_retiming_enabled
                    && (scheduled_precontact || admission_pending)
                {
                    let landing_anchor = if self.precontact_planned[target] {
                        self.precontact_anchor_world[target]
                    } else {
                        prospective_precontact_anchor
                            .expect("scheduled precontact must have a prospective anchor")
                    };
                    let source_ticks_to_touchdown = precontact_offset
                        .map(|offset| (offset as f64 - reference_fraction).max(0.0))
                        .unwrap_or(0.0);
                    let retiming = touchdown_phase_retiming(
                        TouchdownPhaseRetimingInput {
                            source_ticks_to_touchdown,
                            position_error_m: (transition_position - landing_anchor).norm(),
                            tangential_speed_mps: maximum_patch_tangential_speed,
                            normal_speed_mps: transition_velocity.z.abs(),
                            dt_seconds,
                            maximum_acceleration_mps2: self.precontact_maximum_acceleration,
                        },
                        self.support_transition_config,
                        self.touchdown_phase_retiming_config,
                    )
                    .ok_or_else(|| {
                        PyValueError::new_err("touchdown phase-retiming inputs are invalid")
                    })?;
                    phase_target_rate = phase_target_rate.min(retiming.target_phase_rate);
                    target_phase_required_time_seconds = retiming.required_time_seconds;
                    phase_required_time_seconds =
                        phase_required_time_seconds.max(retiming.required_time_seconds);
                    phase_flags |= u8::from(retiming.position_limited)
                        | (u8::from(retiming.tangential_limited) << 1)
                        | (u8::from(retiming.normal_limited) << 2)
                        | (u8::from(retiming.held_at_contact_edge) << 3);
                }
                let touchdown_admitted = if admission_pending {
                    self.support_transition_config
                        .accepts_touchdown(
                            (transition_position - self.precontact_anchor_world[target]).norm(),
                            maximum_patch_tangential_speed,
                            transition_velocity.z.abs(),
                        )
                        .map_err(value_error)?
                } else {
                    true
                };
                let hold_existing_support = !contact_requested
                    && prior_phase.is_contact()
                    && has_pending_requested_support
                    && !has_established_requested_support;
                let is_contact = !self.no_contact_safe_mode
                    && !contact_release_suppressed
                    && ((contact_requested && touchdown_admitted) || hold_existing_support);
                let is_precontact = !self.no_contact_safe_mode
                    && !contact_release_suppressed
                    && (scheduled_precontact || (admission_pending && !touchdown_admitted));
                precontact_transition |= is_precontact;
                let tangential_speed = (is_contact && prior_phase == SupportPhase::TouchdownNormal)
                    .then_some(maximum_patch_tangential_speed);
                support_tangential_speed_out[[tick, target]] = if material_state_required {
                    maximum_patch_tangential_speed
                } else {
                    f64::NAN
                };
                support_touchdown_position_error_out[[tick, target]] =
                    if material_state_required && self.precontact_planned[target] {
                        (transition_position - self.precontact_anchor_world[target]).norm()
                    } else {
                        f64::NAN
                    };
                support_touchdown_normal_speed_out[[tick, target]] = if material_state_required {
                    transition_velocity.z.abs()
                } else {
                    f64::NAN
                };
                let (entered_contact, phase_changed) = if tick == 0 && is_contact {
                    self.support_transitions[target].initialize_locked();
                    (true, true)
                } else {
                    let observation = self.support_transitions[target]
                        .advance(
                            is_contact,
                            is_precontact,
                            tangential_speed,
                            self.support_transition_config,
                        )
                        .map_err(value_error)?;
                    (observation.entered_contact, observation.phase_changed)
                };
                if phase_changed
                    && self.support_transitions[target].phase == SupportPhase::Precontact
                {
                    self.precontact_planned[target] = true;
                    let offset = precontact_offset.expect("precontact phase requires lookahead");
                    let authored_anchor = prospective_precontact_anchor
                        .expect("precontact phase requires a prospective anchor");
                    self.precontact_authored_anchor_world[target] = authored_anchor;
                    self.precontact_anchor_world[target] = authored_anchor;
                    self.precontact_future_root_world[target] = Vec3::new(
                        root_target_positions[[reference_tick + offset, 0]],
                        root_target_positions[[reference_tick + offset, 1]],
                        root_target_positions[[reference_tick + offset, 2]],
                    );
                    self.precontact_rotation_world[target] =
                        self.tracking_cache.world_from_body[frame.0].rotation;
                }
                if entered_contact {
                    self.contact_anchor_world[target] = current_position;
                    let pose = self.tracking_cache.world_from_body[frame.0];
                    for point in 0..self.contact_points_per_target {
                        self.contact_patch_anchor_world[target][point] = pose
                            .transform_point(&Point3::from(self.contact_patch_points[point]))
                            .coords;
                    }
                }
                let support_phase = self.support_transitions[target].phase;
                if support_phase == SupportPhase::Swing {
                    self.precontact_planned[target] = false;
                }
                if self.precontact_planned[target] && support_phase == SupportPhase::Precontact {
                    let landing_retarget_frozen = precontact_offset
                        .is_none_or(|offset| offset <= self.capture_landing_freeze_ticks);
                    if self.capture_landing_retarget_enabled
                        && self.last_dcm_observation_valid
                        && !landing_retarget_frozen
                    {
                        let retarget = capture_landing_retarget(
                            self.precontact_authored_anchor_world[target],
                            self.precontact_anchor_world[target],
                            self.precontact_future_root_world[target],
                            self.last_dcm_world,
                            self.last_dcm_support_margin_m,
                            dt_seconds,
                            self.capture_landing_retarget_config,
                        )
                        .ok_or_else(|| {
                            PyValueError::new_err(
                                "capture landing retarget inputs or reach envelope are invalid",
                            )
                        })?;
                        self.precontact_anchor_world[target] = retarget.applied_anchor_world;
                        landing_retarget_capture_scale_out[[tick, target]] = retarget.capture_scale;
                        landing_retarget_offset_out[[tick, target]] = retarget.authored_offset_m;
                        landing_retarget_reach_out[[tick, target]] =
                            retarget.root_to_landing_reach_m;
                        landing_retarget_flags_out[[tick, target]] =
                            u8::from(retarget.authored_offset_was_limited)
                                | (u8::from(retarget.reach_was_limited) << 1)
                                | (u8::from(retarget.slew_was_limited) << 2)
                                | (u8::from(!retarget.reach_envelope_was_feasible) << 3);
                    }
                    if self.capture_landing_retarget_enabled && landing_retarget_frozen {
                        landing_retarget_flags_out[[tick, target]] |= 1 << 4;
                    }
                    for axis in 0..3 {
                        landing_retarget_anchor_out[[tick, target, axis]] =
                            self.precontact_anchor_world[target][axis];
                    }
                    landing_retarget_offset_out[[tick, target]] = (self.precontact_anchor_world
                        [target]
                        - self.precontact_authored_anchor_world[target])
                        .fixed_rows::<2>(0)
                        .norm();
                    landing_retarget_reach_out[[tick, target]] = (self.precontact_anchor_world
                        [target]
                        - self.precontact_future_root_world[target])
                        .norm();
                }
                if self.precontact_planned[target]
                    && matches!(
                        support_phase,
                        SupportPhase::Precontact | SupportPhase::TouchdownNormal
                    )
                {
                    if !self.angular_tasks.is_empty() {
                        return Err(PyValueError::new_err(
                            "more than one simultaneous precontact angular task is unsupported",
                        ));
                    }
                    self.program
                        .model
                        .floating_angular_jacobian_into(
                            &self.tracking_cache,
                            frame,
                            &mut self.tracking_jacobian,
                        )
                        .map_err(value_error)?;
                    let angular_velocity = Vec3::from_fn(|axis, _| {
                        (0..self.generalized_velocity.len())
                            .map(|coordinate| {
                                self.tracking_jacobian[(axis, coordinate)]
                                    * self.generalized_velocity[coordinate]
                            })
                            .sum()
                    });
                    let rotation = self.tracking_cache.world_from_body[frame.0].rotation;
                    let local_error = rotation
                        .rotation_to(&self.precontact_rotation_world[target])
                        .scaled_axis();
                    let error_world = rotation.transform_vector(&local_error);
                    let desired_angular_acceleration = clamp_norm(
                        self.point_omega * self.point_omega * error_world
                            - 2.0 * self.point_omega * angular_velocity,
                        self.precontact_maximum_acceleration,
                    );
                    self.angular_tasks
                        .push(FloatingFrameAngularAccelerationTask {
                            stable_id: 50 + target as u32,
                            frame,
                            desired_angular_acceleration_world: desired_angular_acceleration,
                            priority: Priority::Viability,
                            weight: weights[target],
                        });
                }
                let (tracking_position, tracking_velocity, tracking_feedforward) = if is_contact {
                    (
                        self.contact_anchor_world[target],
                        Vec3::zeros(),
                        Vec3::zeros(),
                    )
                } else {
                    (target_position, target_velocity, target_acceleration)
                };
                let desired = if is_precontact {
                    // Once the scheduled edge has passed, continue shaping
                    // toward the stored landing anchor with a bounded receding
                    // horizon. This prevents one stale schedule sample from
                    // manufacturing a mid-air contact.
                    let time_to_touchdown_seconds = if self.reference_phase_retiming_enabled {
                        let source_ticks_to_touchdown = precontact_offset
                            .map(|offset| (offset as f64 - reference_fraction).max(0.0))
                            .unwrap_or(0.0);
                        let scheduled_time = if phase_rate > 1e-9 {
                            source_ticks_to_touchdown * dt_seconds / phase_rate
                        } else {
                            0.0
                        };
                        scheduled_time
                            .max(target_phase_required_time_seconds)
                            .max(dt_seconds)
                    } else {
                        precontact_offset.unwrap_or_else(|| self.precontact_ticks.clamp(1, 60))
                            as f64
                            * dt_seconds
                    };
                    cubic_precontact_acceleration(
                        transition_position,
                        transition_velocity,
                        self.precontact_anchor_world[target],
                        Vec3::zeros(),
                        time_to_touchdown_seconds,
                        self.precontact_maximum_acceleration,
                    )
                    .map_err(value_error)?
                } else {
                    tracking_acceleration(
                        tracking_position,
                        tracking_velocity,
                        tracking_feedforward,
                        current_position,
                        current_velocity,
                        if is_contact {
                            contact_omega
                        } else {
                            self.point_omega
                        },
                        if is_contact { 25.0 } else { 150.0 },
                    )
                };
                if is_contact {
                    let first_contact = self.contacts.len();
                    for point in 0..self.contact_points_per_target {
                        let normal_only = support_phase.is_normal_only();
                        let (mode, kinematic_enabled) = if self.contact_points_per_target == 4 {
                            if normal_only {
                                (ContactMode::NormalPoint, point < 3)
                            } else {
                                match point {
                                    0 => (ContactMode::LockedPoint, true),
                                    1 => (ContactMode::NormalPoint, true),
                                    2 => (ContactMode::LockedPoint, false),
                                    3 => (ContactMode::RollingPoint, true),
                                    _ => unreachable!(),
                                }
                            }
                        } else {
                            (
                                if normal_only {
                                    ContactMode::NormalPoint
                                } else {
                                    ContactMode::LockedPoint
                                },
                                true,
                            )
                        };
                        let mut contact = ContactSpec::horizontal(
                            1 + (target * 4 + point) as u32,
                            frame,
                            self.contact_patch_points[point],
                            mode,
                            self.contact_friction_coefficient,
                            self.maximum_normal_force_multiple * self.supported_weight,
                            0.0,
                        );
                        contact.kinematic_enabled = kinematic_enabled;
                        if kinematic_enabled {
                            self.program
                                .model
                                .floating_point_jacobian_into(
                                    &self.tracking_cache,
                                    frame,
                                    self.contact_patch_points[point],
                                    &mut self.tracking_jacobian,
                                )
                                .map_err(value_error)?;
                            let patch_position = self.tracking_cache.world_from_body[frame.0]
                                .transform_point(&Point3::from(self.contact_patch_points[point]))
                                .coords;
                            let patch_velocity = Vec3::from_fn(|axis, _| {
                                (0..self.generalized_velocity.len())
                                    .map(|coordinate| {
                                        self.tracking_jacobian[(axis, coordinate)]
                                            * self.generalized_velocity[coordinate]
                                    })
                                    .sum()
                            });
                            contact.desired_point_acceleration_world = tracking_acceleration(
                                self.contact_patch_anchor_world[target][point],
                                Vec3::zeros(),
                                Vec3::zeros(),
                                patch_position,
                                patch_velocity,
                                self.point_omega,
                                50.0,
                            );
                        }
                        self.contacts.push(contact);
                    }
                    if self.contact_points_per_target == 4
                        && self.minimum_contact_cop_margin_m > 0.0
                    {
                        self.support_patches.push(SupportPatchSpec {
                            stable_id: 1 + target as u32,
                            first_contact,
                            contact_count: 4,
                            minimum_margin_m: self.minimum_contact_cop_margin_m,
                        });
                    }
                }
                if is_contact || weights[target] > 0.0 {
                    let (task_point_in_frame, task_desired_acceleration) = if is_precontact {
                        (transition_point, desired)
                    } else if is_contact
                        && support_phase.is_normal_only()
                        && (self.material_touchdown_task
                            || (self.precontact_planned[target]
                                && support_phase == SupportPhase::TouchdownNormal))
                    {
                        let transition_point = if self.contact_points_per_target == 4 {
                            (self.contact_patch_points[0] + self.contact_patch_points[3]) * 0.5
                        } else {
                            self.contact_patch_points[0]
                        };
                        self.program
                            .model
                            .floating_point_jacobian_into(
                                &self.tracking_cache,
                                frame,
                                transition_point,
                                &mut self.tracking_jacobian,
                            )
                            .map_err(value_error)?;
                        let transition_position = self.tracking_cache.world_from_body[frame.0]
                            .transform_point(&Point3::from(transition_point))
                            .coords;
                        let transition_velocity = Vec3::from_fn(|axis, _| {
                            (0..self.generalized_velocity.len())
                                .map(|coordinate| {
                                    self.tracking_jacobian[(axis, coordinate)]
                                        * self.generalized_velocity[coordinate]
                                })
                                .sum()
                        });
                        let transition_anchor = if self.contact_points_per_target == 4 {
                            (self.contact_patch_anchor_world[target][0]
                                + self.contact_patch_anchor_world[target][3])
                                * 0.5
                        } else {
                            self.contact_patch_anchor_world[target][0]
                        };
                        (
                            transition_point,
                            tracking_acceleration(
                                transition_anchor,
                                Vec3::zeros(),
                                Vec3::zeros(),
                                transition_position,
                                transition_velocity,
                                contact_omega,
                                25.0,
                            ),
                        )
                    } else {
                        (Vec3::zeros(), desired)
                    };
                    self.point_tasks.push(FloatingPointAccelerationTask {
                        stable_id: 10 + target as u32,
                        frame,
                        point_in_frame: task_point_in_frame,
                        desired_acceleration_world: task_desired_acceleration,
                        priority: if is_contact || is_precontact {
                            Priority::Viability
                        } else {
                            point_priorities[target]
                        },
                        weight: if is_contact && !support_phase.is_normal_only() {
                            0.0
                        } else {
                            weights[target]
                        },
                    });
                }
            }
            if self.contacts.len() > self.maximum_contacts {
                return Err(PyValueError::new_err(
                    "floating trace contact patch exceeds session contact capacity",
                ));
            }
            if !self.contacts.is_empty() {
                let nominal_normal_force = self.supported_weight / self.contacts.len() as f64;
                for contact in &mut self.contacts {
                    contact.nominal_normal_force = nominal_normal_force;
                }
            }
            let mut desired_center_of_mass = desired_center_of_mass_tracking;
            for axis in 0..3 {
                dcm_out[[tick, axis]] = f64::NAN;
                target_dcm_out[[tick, axis]] = f64::NAN;
                virtual_zmp_out[[tick, axis]] = f64::NAN;
                clipped_zmp_out[[tick, axis]] = f64::NAN;
            }
            dcm_natural_frequency_out[tick] = f64::NAN;
            dcm_measured_height_out[tick] = f64::NAN;
            dcm_height_clamped_out[tick] = 0;
            dcm_zmp_clipped_out[tick] = 0;
            dcm_support_vertices_out[tick] = 0;
            dcm_support_margin_out[tick] = f64::NAN;
            if self.dcm_balance_enabled {
                let mut support_points = [Vec3::zeros(); 16];
                let mut support_point_count = 0usize;
                for target in 0..target_count {
                    if !self.support_transitions[target].phase.is_contact() {
                        continue;
                    }
                    for point in 0..self.contact_points_per_target {
                        support_points[support_point_count] =
                            self.contact_patch_anchor_world[target][point];
                        support_point_count += 1;
                    }
                }
                // A released/failed contact leaves no measured support
                // polygon.  DCM is an authority observer, not a reason to
                // abort the whole trace; fall back to the authored CoM task
                // until a nondegenerate measured polygon returns.
                if support_point_count >= 3 {
                    if let Some(balance) = dcm_balance_acceleration(
                        center_of_mass_world,
                        center_of_mass_velocity_world,
                        center_of_mass_target_position,
                        center_of_mass_target_velocity,
                        &support_points[..support_point_count],
                        self.dcm_balance_config,
                    ) {
                        desired_center_of_mass.x = balance.desired_horizontal_acceleration_world.x;
                        desired_center_of_mass.y = balance.desired_horizontal_acceleration_world.y;
                        for axis in 0..3 {
                            dcm_out[[tick, axis]] = balance.dcm_world[axis];
                            target_dcm_out[[tick, axis]] = balance.target_dcm_world[axis];
                            virtual_zmp_out[[tick, axis]] = balance.virtual_zmp_world[axis];
                            clipped_zmp_out[[tick, axis]] = balance.clipped_zmp_world[axis];
                        }
                        dcm_natural_frequency_out[tick] = balance.natural_frequency_rad_s;
                        dcm_measured_height_out[tick] = balance.measured_com_height_m;
                        dcm_height_clamped_out[tick] = u8::from(balance.com_height_was_clamped);
                        dcm_zmp_clipped_out[tick] = u8::from(balance.zmp_was_clipped);
                        dcm_support_vertices_out[tick] = balance.support_vertex_count as u8;
                        dcm_support_margin_out[tick] = balance.dcm_support_margin_m;
                        self.last_dcm_world = balance.dcm_world;
                        self.last_dcm_support_margin_m = balance.dcm_support_margin_m;
                        self.last_dcm_observation_valid = true;
                    } else {
                        self.last_dcm_observation_valid = false;
                    }
                } else {
                    self.last_dcm_observation_valid = false;
                }
            } else {
                self.last_dcm_observation_valid = false;
            }
            if self.balance_phase_retiming_enabled
                && contact_phase_authority.phase != bonesaw_core::MeasuredContactPhase::MultiSupport
                && dcm_support_margin_out[tick].is_finite()
            {
                let balance_rate = support_margin_phase_rate(
                    dcm_support_margin_out[tick],
                    self.balance_phase_hold_margin_m,
                    self.balance_phase_full_rate_margin_m,
                )
                .ok_or_else(|| PyValueError::new_err("balance phase-retiming inputs are invalid"))?
                .max(self.touchdown_phase_retiming_config.minimum_phase_rate);
                if balance_rate < phase_target_rate {
                    phase_target_rate = balance_rate;
                    phase_flags |= 1 << 6;
                }
            }
            let precontact_age_ticks = self
                .support_transitions
                .iter()
                .filter(|state| state.phase == SupportPhase::Precontact)
                .map(|state| state.phase_ticks)
                .max()
                .unwrap_or(0);
            let authority_target = if self.balance_feedback_authority_enabled
                && contact_phase_authority.phase == bonesaw_core::MeasuredContactPhase::Precontact
            {
                balance_feedback_authority(
                    dcm_support_margin_out[tick],
                    rotation_error_world.norm(),
                    precontact_age_ticks,
                    self.balance_feedback_authority_config,
                )
                .ok_or_else(|| {
                    PyValueError::new_err("balance-feedback authority inputs are invalid")
                })?
                .joint_velocity_envelope_scale
            } else {
                contact_phase_authority.joint_velocity_envelope_scale
            };
            joint_velocity_envelope_target_scale_out[tick] = authority_target;
            self.contact_phase_authority_scale = if authority_target
                >= self.contact_phase_authority_scale
            {
                authority_target
            } else if self
                .contact_phase_authority_maximum_delta_per_tick
                .is_finite()
            {
                slew_contact_phase_authority(
                    self.contact_phase_authority_scale,
                    authority_target,
                    self.contact_phase_authority_maximum_delta_per_tick,
                )
                .ok_or_else(|| PyValueError::new_err("contact-phase authority slew is invalid"))?
            } else {
                authority_target
            };
            joint_velocity_envelope_scale_out[tick] = self.contact_phase_authority_scale;
            self.velocity_envelope_coordinates.clear();
            self.velocity_envelope_accelerations.clear();
            if self.joint_velocity_envelope_weight > 0.0 && self.contact_phase_authority_scale > 0.0
            {
                for coordinate in 0..dof {
                    let Some(acceleration) = joint_velocity_envelope_acceleration(
                        self.state.robot.v[coordinate],
                        self.joint_velocity_limits[coordinate],
                        self.joint_velocity_envelope_activation_fraction,
                        self.joint_velocity_envelope_omega,
                        self.maximum_acceleration,
                    ) else {
                        continue;
                    };
                    if acceleration != 0.0 {
                        self.velocity_envelope_coordinates.push(coordinate);
                        self.velocity_envelope_accelerations.push(acceleration);
                    }
                }
            }
            joint_velocity_envelope_active_coordinates_out[tick] =
                self.velocity_envelope_coordinates.len() as u8;
            let velocity_envelope_task = (!self.velocity_envelope_coordinates.is_empty())
                .then_some(FloatingJointAccelerationTask {
                    coordinates: &self.velocity_envelope_coordinates,
                    desired_accelerations: &self.velocity_envelope_accelerations,
                    priority: self.joint_velocity_envelope_priority,
                    weight: self.joint_velocity_envelope_weight
                        * self.contact_phase_authority_scale,
                });
            let joint_acceleration_task = if protected_joint_task.is_some() {
                protected_joint_task
            } else {
                velocity_envelope_task
            };
            for axis in 0..3 {
                center_of_mass_command_out[[tick, axis]] = desired_center_of_mass[axis];
            }
            let center_of_mass_task =
                (self.center_of_mass_task_weight > 0.0).then_some(FloatingCenterOfMassTask {
                    desired_acceleration_world: desired_center_of_mass,
                    horizontal_only: self.dcm_balance_enabled,
                    priority: self.center_of_mass_task_priority,
                    weight: self.center_of_mass_task_weight,
                });
            self.point_tasks.sort_unstable_by_key(|task| task.stable_id);
            let touchdown_transition = self
                .support_transitions
                .iter()
                .any(|state| state.phase == SupportPhase::TouchdownNormal);
            let mut normal_contact_contingency = self
                .support_transitions
                .iter()
                .any(|state| state.phase == SupportPhase::NormalFallback);
            let mut contact_release_contingency = false;
            let mut no_contact_safe_fallback = false;
            if self.no_contact_safe_mode && self.contacts.is_empty() {
                // The controller has already entered the bounded free-body
                // fallback.  Do not pay the dense feasibility cost again
                // while the authored schedule remains contact-free.
                self.output.status = SolveStatus::MaxIterations;
            } else {
                let solve_input = FloatingDynamicWbcInput {
                    state: &self.state.robot,
                    root_twist_world: self.state.root_twist_world,
                    desired_generalized_acceleration: &self.desired_acceleration,
                    task_priorities: FloatingTaskPriorities {
                        root_angular: Priority::Invariant,
                        root_horizontal: self.root_horizontal_task_priority,
                        root_height: Priority::Invariant,
                        joint_posture: self.joint_posture_priority,
                    },
                    task_weights: FloatingTaskWeights {
                        root_angular: self.root_angular_task_weight,
                        root_horizontal: self.root_horizontal_task_weight,
                        root_height: self.root_height_task_weight,
                        joint_posture: 1.0,
                    },
                    joint_posture_weight: self.joint_posture_weight,
                    joint_acceleration_task,
                    center_of_mass_task,
                    centroidal_angular_momentum_task,
                    frame_angular_acceleration_tasks: &self.angular_tasks,
                    point_acceleration_tasks: &self.point_tasks,
                    generalized_acceleration_bounds: &self.acceleration_bounds,
                    torque_bounds: &self.torque_bounds,
                    actuator_effort: self.coupled_actuation_enabled.then_some(
                        ActuatorEffortInput {
                            actuation: &self.program.actuation,
                            bounds: &self.actuator_effort_bounds,
                        },
                    ),
                    contacts: &self.contacts,
                    support_patches: &self.support_patches,
                };
                self.controller
                    .solve_into(solve_input, &mut self.output, &mut self.scratch)
                    .map_err(value_error)?;
            }
            // Any non-solved contact result is unsafe to integrate with the
            // authored hard rows.  Demote once to normal-only rows, then
            // release the contact if the bounded solver still cannot produce
            // an executable result.  In particular, MaxIterations must not
            // leave the trace frozen behind an apparently healthy contact.
            let contact_solve_unsolved = !matches!(
                self.output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            );
            if contact_solve_unsolved && !self.contacts.is_empty() && !normal_contact_contingency {
                for (point, contact) in self.contacts.iter_mut().enumerate() {
                    let fallback_kinematic = self.contact_points_per_target == 1 || point % 4 < 3;
                    contact.mode = ContactMode::NormalPoint;
                    contact.kinematic_enabled = fallback_kinematic;
                    let target = (contact.stable_id - 1) as usize / 4;
                    self.support_transitions[target].mark_normal_fallback();
                    if let Some(task) = self
                        .point_tasks
                        .iter_mut()
                        .find(|task| task.stable_id == 10 + target as u32)
                    {
                        task.priority = Priority::Viability;
                        task.weight = weights[target];
                    }
                }
                normal_contact_contingency = true;
                self.controller
                    .solve_into(
                        FloatingDynamicWbcInput {
                            state: &self.state.robot,
                            root_twist_world: self.state.root_twist_world,
                            desired_generalized_acceleration: &self.desired_acceleration,
                            task_priorities: FloatingTaskPriorities {
                                root_angular: Priority::Invariant,
                                root_horizontal: self.root_horizontal_task_priority,
                                root_height: Priority::Invariant,
                                joint_posture: self.joint_posture_priority,
                            },
                            task_weights: FloatingTaskWeights {
                                root_angular: self.root_angular_task_weight,
                                root_horizontal: self.root_horizontal_task_weight,
                                root_height: self.root_height_task_weight,
                                joint_posture: 1.0,
                            },
                            joint_posture_weight: self.joint_posture_weight,
                            joint_acceleration_task,
                            center_of_mass_task,
                            centroidal_angular_momentum_task,
                            frame_angular_acceleration_tasks: &self.angular_tasks,
                            point_acceleration_tasks: &self.point_tasks,
                            generalized_acceleration_bounds: &self.acceleration_bounds,
                            torque_bounds: &self.torque_bounds,
                            actuator_effort: self.coupled_actuation_enabled.then_some(
                                ActuatorEffortInput {
                                    actuation: &self.program.actuation,
                                    bounds: &self.actuator_effort_bounds,
                                },
                            ),
                            contacts: &self.contacts,
                            support_patches: &self.support_patches,
                        },
                        &mut self.output,
                        &mut self.scratch,
                    )
                    .map_err(value_error)?;
            }
            if !matches!(
                self.output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ) && !self.contacts.is_empty()
            {
                for contact in &self.contacts {
                    let target = contact.stable_id.saturating_sub(1) as usize / 4;
                    if target < self.contact_release_suppressed.len() {
                        self.contact_release_suppressed[target] = true;
                        self.support_transitions[target].clear();
                        self.precontact_planned[target] = false;
                        self.precontact_authored_anchor_world[target] = Vec3::zeros();
                        self.precontact_anchor_world[target] = Vec3::zeros();
                        self.precontact_future_root_world[target] = Vec3::zeros();
                        self.precontact_rotation_world[target] = UnitQuaternion::identity();
                        self.contact_anchor_world[target] = Vec3::zeros();
                        self.contact_patch_anchor_world[target] = [Vec3::zeros(); 4];
                    }
                }
                self.contacts.clear();
                self.support_patches.clear();
                contact_release_contingency = true;
                self.no_contact_safe_mode = true;
            }
            // A contact-free unsolved result is recoverable.  Enter a
            // deterministic free-body fallback instead of paying the dense
            // feasibility cost again and leaving the interactive state frozen.
            // The fallback is deliberately conservative: damp angular and
            // joint velocity, apply gravity to the free root, clamp every
            // component to the already-admitted acceleration envelope, and
            // keep the typed contingency status visible to the caller.
            if !matches!(
                self.output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ) && self.contacts.is_empty()
            {
                self.point_tasks.clear();
                self.angular_tasks.clear();
                let root_twist = self.state.root_twist_world.0.as_slice();
                self.output.generalized_acceleration.fill(0.0);
                for coordinate in 0..generalized_dof {
                    let candidate = if coordinate < 3 {
                        -2.0 * root_twist[coordinate] / dt_seconds
                    } else if coordinate < 6 {
                        if coordinate == 5 { -9.81 } else { 0.0 }
                    } else {
                        -2.0 * self.state.robot.v[coordinate - 6] / dt_seconds
                    };
                    let lower = self.acceleration_bounds.lower[coordinate];
                    let upper = self.acceleration_bounds.upper[coordinate];
                    self.output.generalized_acceleration[coordinate] = if lower <= upper {
                        candidate.clamp(lower, upper)
                    } else {
                        0.0
                    };
                }
                self.output.contact_force_basis.fill(0.0);
                self.output.contact_force_world.fill(0.0);
                self.output.active_contacts = 0;
                // The rejected contact solve may have left diagnostic values
                // from its hard rows in the reusable output workspace.  The
                // fallback has no contact, torque, support, or collision
                // witness; never expose those stale values as if they were
                // measured authority in the next browser/plant snapshot.
                self.output.actuator_torque.fill(0.0);
                self.output.dynamics_residual_linf = 0.0;
                self.output.contact_acceleration_residual_linf = 0.0;
                self.output.minimum_friction_margin = f64::INFINITY;
                self.output.minimum_support_margin_m = f64::INFINITY;
                self.output.limiting_support_patch = None;
                self.output.minimum_torque_margin = f64::INFINITY;
                self.output.minimum_actuator_effort_margin = f64::INFINITY;
                self.output.limiting_actuator = None;
                self.output.collision_barrier =
                    bonesaw_core::CollisionAccelerationBarrierEvidence::disabled();
                self.output.world_collision_barrier =
                    bonesaw_core::WorldCollisionBarrierEvidence::disabled();
                self.output.solve.status = SolveStatus::MaxIterations;
                self.output.solve.level_residuals.clear();
                self.output.solve.clipped_levels.clear();
                self.output.solve.rank_by_level.clear();
                self.output.solve.active_constraints.clear();
                self.output.solve.task_pseudoinverse_calls = 0;
                self.output.solve.task_pseudoinverse_calls_by_level.fill(0);
                self.output.solve.task_jacobi_sweeps = 0;
                self.output.solve.task_jacobi_sweeps_by_level.fill(0);
                self.output.solve.clipped_steps = 0;
                self.output.solve.clipped_steps_by_level.fill(0);
                self.output.solve.equality_pseudoinverse_reused = false;
                self.output.solve.feasibility_projection_sweeps = 0;
                self.output.solve.feasibility_halfspace_projections = 0;
                self.output.solve.feasibility_polish_iterations = 0;
                self.output.solve.feasibility_polish_pseudoinverse_calls = 0;
                self.output.solve.feasibility_polish_jacobi_sweeps = 0;
                self.output.solve.feasibility_seed_reused = false;
                self.output.solve.feasibility_prefix_resumed = false;
                for residual in &mut self.output.task_residuals {
                    residual.active = false;
                    residual.clipped = false;
                    residual.rows = 0;
                    residual.l2 = 0.0;
                    residual.rms = 0.0;
                }
                self.no_contact_safe_mode = true;
                no_contact_safe_fallback = true;
            }
            let solved = matches!(
                self.output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ) || no_contact_safe_fallback;
            if solved {
                self.program
                    .model
                    .integrate_floating(
                        &mut self.state,
                        &self.output.generalized_acceleration,
                        dt_seconds,
                    )
                    .map_err(value_error)?;
            }
            self.program
                .model
                .forward_kinematics(&self.state.robot, &mut self.tracking_cache)
                .map_err(value_error)?;

            let root_translation = self.state.robot.control_world_from_root.translation.vector;
            for axis in 0..3 {
                root_translation_out[[tick, axis]] = root_translation[axis];
            }
            let quaternion = self
                .state
                .robot
                .control_world_from_root
                .rotation
                .quaternion();
            root_quaternion_wxyz_out[[tick, 0]] = quaternion.w;
            root_quaternion_wxyz_out[[tick, 1]] = quaternion.i;
            root_quaternion_wxyz_out[[tick, 2]] = quaternion.j;
            root_quaternion_wxyz_out[[tick, 3]] = quaternion.k;
            for coordinate in 0..dof {
                q_out[[tick, coordinate]] = self.state.robot.q[coordinate];
                v_out[[tick, coordinate]] = self.state.robot.v[coordinate];
            }
            for target in 0..target_count {
                let frame = frame_ids[target] as usize;
                let translation = self.tracking_cache.world_from_body[frame]
                    .translation
                    .vector;
                for axis in 0..3 {
                    tracked_positions_out[[tick, target, axis]] = translation[axis];
                }
                support_phase_out[[tick, target]] = self.support_transitions[target].phase as u8;
                support_phase_ticks_out[[tick, target]] = self.support_transitions[target]
                    .phase_ticks
                    .min(u16::MAX as usize)
                    as u16;
            }
            for slot in 0..self.maximum_contacts {
                contact_normal_force_out[[tick, slot]] = self.output.contact_force_basis[(2, slot)];
            }
            for slot in 0..FLOATING_TASK_DIAGNOSTIC_CAPACITY {
                task_rms_out[[tick, slot]] = self.output.task_residuals[slot].rms;
            }
            dynamics_residual_out[tick] = self.output.dynamics_residual_linf;
            contact_residual_out[tick] = self.output.contact_acceleration_residual_linf;
            minimum_support_margin_out[tick] = self.output.minimum_support_margin_m;
            status_out[tick] = match self.output.status {
                _ if contact_release_contingency => 5,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
                    if normal_contact_contingency =>
                {
                    4
                }
                SolveStatus::Solved | SolveStatus::SolvedWithSlack if no_contact_safe_fallback => 4,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack if touchdown_transition => 6,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack if precontact_transition => 7,
                SolveStatus::Solved => 0,
                SolveStatus::SolvedWithSlack => 1,
                SolveStatus::PrimalInfeasible => 2,
                SolveStatus::NumericalFailure | SolveStatus::InvalidProblem => 3,
                SolveStatus::MaxIterations => 4,
            };
            task_pseudoinverse_calls_out[tick] = self
                .output
                .solve
                .task_pseudoinverse_calls
                .min(u16::MAX as usize) as u16;
            clipped_steps_out[tick] = self.output.solve.clipped_steps.min(u16::MAX as usize) as u16;
            task_jacobi_sweeps_out[tick] =
                self.output.solve.task_jacobi_sweeps.min(u16::MAX as usize) as u16;
            feasibility_projection_sweeps_out[tick] =
                self.output
                    .solve
                    .feasibility_projection_sweeps
                    .min(u16::MAX as usize) as u16;
            feasibility_halfspace_projections_out[tick] =
                self.output
                    .solve
                    .feasibility_halfspace_projections
                    .min(u32::MAX as usize) as u32;
            feasibility_polish_iterations_out[tick] =
                self.output
                    .solve
                    .feasibility_polish_iterations
                    .min(u16::MAX as usize) as u16;
            feasibility_polish_pseudoinverse_calls_out[tick] =
                self.output
                    .solve
                    .feasibility_polish_pseudoinverse_calls
                    .min(u16::MAX as usize) as u16;
            feasibility_polish_jacobi_sweeps_out[tick] =
                self.output
                    .solve
                    .feasibility_polish_jacobi_sweeps
                    .min(u16::MAX as usize) as u16;
            for priority in Priority::ALL {
                task_pseudoinverse_calls_by_level_out[[tick, priority as usize]] =
                    self.output.solve.task_pseudoinverse_calls_by_level[priority as usize]
                        .min(u16::MAX as usize) as u16;
                clipped_steps_by_level_out[[tick, priority as usize]] =
                    self.output.solve.clipped_steps_by_level[priority as usize]
                        .min(u16::MAX as usize) as u16;
                task_jacobi_sweeps_by_level_out[[tick, priority as usize]] =
                    self.output.solve.task_jacobi_sweeps_by_level[priority as usize]
                        .min(u16::MAX as usize) as u16;
            }
            reference_phase_target_rate_out[tick] = phase_target_rate;
            reference_phase_required_time_out[tick] = phase_required_time_seconds;
            if phase_target_rate < 1.0 {
                phase_flags |= 1 << 4;
            }
            if reference_phase >= ticks.saturating_sub(1) as f64 {
                phase_flags |= 1 << 5;
            }
            reference_phase_flags_out[tick] = phase_flags;
            if self.reference_phase_retiming_enabled {
                let next_phase_rate = slew_touchdown_phase_rate(
                    self.reference_phase_rate,
                    phase_target_rate,
                    self.touchdown_phase_retiming_config,
                )
                .ok_or_else(|| PyValueError::new_err("touchdown phase-rate slew is invalid"))?;
                self.previous_reference_phase_rate = self.reference_phase_rate;
                self.reference_phase_rate = next_phase_rate;
                self.reference_phase =
                    (reference_phase + next_phase_rate).min(ticks.saturating_sub(1) as f64);
            }
            step_ns_out[tick] = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        }
        Ok(())
    }
}

#[allow(clippy::too_many_arguments)]
fn sample_array2_vector_jet(
    positions: &ArrayView2<'_, f64>,
    velocities: &ArrayView2<'_, f64>,
    accelerations: &ArrayView2<'_, f64>,
    tick: usize,
    next_tick: usize,
    fraction: f64,
    dt_seconds: f64,
    phase_rate: f64,
    phase_acceleration: f64,
) -> PyResult<VectorJet> {
    let start = VectorJet {
        value: Vec3::new(
            positions[[tick, 0]],
            positions[[tick, 1]],
            positions[[tick, 2]],
        ),
        velocity: Vec3::new(
            velocities[[tick, 0]],
            velocities[[tick, 1]],
            velocities[[tick, 2]],
        ),
        acceleration: Vec3::new(
            accelerations[[tick, 0]],
            accelerations[[tick, 1]],
            accelerations[[tick, 2]],
        ),
    };
    let end = VectorJet {
        value: Vec3::new(
            positions[[next_tick, 0]],
            positions[[next_tick, 1]],
            positions[[next_tick, 2]],
        ),
        velocity: Vec3::new(
            velocities[[next_tick, 0]],
            velocities[[next_tick, 1]],
            velocities[[next_tick, 2]],
        ),
        acceleration: Vec3::new(
            accelerations[[next_tick, 0]],
            accelerations[[next_tick, 1]],
            accelerations[[next_tick, 2]],
        ),
    };
    let authored = sample_quintic_vector_jet(start, end, dt_seconds, fraction)
        .ok_or_else(|| PyValueError::new_err("authored vector jet is invalid"))?;
    time_warp_vector_jet(authored, phase_rate, phase_acceleration)
        .ok_or_else(|| PyValueError::new_err("reference time warp is invalid"))
}

#[allow(clippy::too_many_arguments)]
fn sample_array2_scalar_jet(
    positions: &ArrayView2<'_, f64>,
    velocities: &ArrayView2<'_, f64>,
    accelerations: &ArrayView2<'_, f64>,
    coordinate: usize,
    tick: usize,
    next_tick: usize,
    fraction: f64,
    dt_seconds: f64,
    phase_rate: f64,
    phase_acceleration: f64,
) -> PyResult<ScalarJet> {
    let start = ScalarJet {
        value: positions[[tick, coordinate]],
        velocity: velocities[[tick, coordinate]],
        acceleration: accelerations[[tick, coordinate]],
    };
    let end = ScalarJet {
        value: positions[[next_tick, coordinate]],
        velocity: velocities[[next_tick, coordinate]],
        acceleration: accelerations[[next_tick, coordinate]],
    };
    let authored = sample_quintic_scalar_jet(start, end, dt_seconds, fraction)
        .ok_or_else(|| PyValueError::new_err("authored scalar jet is invalid"))?;
    time_warp_scalar_jet(authored, phase_rate, phase_acceleration)
        .ok_or_else(|| PyValueError::new_err("reference scalar time warp is invalid"))
}

#[allow(clippy::too_many_arguments)]
fn sample_array3_vector_jet(
    positions: &ArrayView3<'_, f64>,
    velocities: &ArrayView3<'_, f64>,
    accelerations: &ArrayView3<'_, f64>,
    target: usize,
    tick: usize,
    next_tick: usize,
    fraction: f64,
    dt_seconds: f64,
    phase_rate: f64,
    phase_acceleration: f64,
) -> PyResult<VectorJet> {
    let start = VectorJet {
        value: Vec3::new(
            positions[[tick, target, 0]],
            positions[[tick, target, 1]],
            positions[[tick, target, 2]],
        ),
        velocity: Vec3::new(
            velocities[[tick, target, 0]],
            velocities[[tick, target, 1]],
            velocities[[tick, target, 2]],
        ),
        acceleration: Vec3::new(
            accelerations[[tick, target, 0]],
            accelerations[[tick, target, 1]],
            accelerations[[tick, target, 2]],
        ),
    };
    let end = VectorJet {
        value: Vec3::new(
            positions[[next_tick, target, 0]],
            positions[[next_tick, target, 1]],
            positions[[next_tick, target, 2]],
        ),
        velocity: Vec3::new(
            velocities[[next_tick, target, 0]],
            velocities[[next_tick, target, 1]],
            velocities[[next_tick, target, 2]],
        ),
        acceleration: Vec3::new(
            accelerations[[next_tick, target, 0]],
            accelerations[[next_tick, target, 1]],
            accelerations[[next_tick, target, 2]],
        ),
    };
    let authored = sample_quintic_vector_jet(start, end, dt_seconds, fraction)
        .ok_or_else(|| PyValueError::new_err("authored point jet is invalid"))?;
    time_warp_vector_jet(authored, phase_rate, phase_acceleration)
        .ok_or_else(|| PyValueError::new_err("point-reference time warp is invalid"))
}

fn clamp_norm(value: Vec3, maximum: f64) -> Vec3 {
    let norm = value.norm();
    if norm > maximum && norm > 0.0 {
        value * (maximum / norm)
    } else {
        value
    }
}

fn tracking_acceleration(
    target_position: Vec3,
    target_velocity: Vec3,
    target_acceleration: Vec3,
    current_position: Vec3,
    current_velocity: Vec3,
    omega: f64,
    maximum_acceleration: f64,
) -> Vec3 {
    clamp_norm(
        target_acceleration
            + 2.0 * omega * (target_velocity - current_velocity)
            + omega * omega * (target_position - current_position),
        maximum_acceleration,
    )
}

fn priority(value: u8) -> PyResult<Priority> {
    match value {
        0 => Ok(Priority::Invariant),
        1 => Ok(Priority::Viability),
        2 => Ok(Priority::Intent),
        3 => Ok(Priority::Preference),
        4 => Ok(Priority::Style),
        _ => Err(PyValueError::new_err("priority must be in 0..=4")),
    }
}

#[pymodule]
fn _bonesaw(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add("__version__", env!("CARGO_PKG_VERSION"))?;
    module.add(
        "DYNAMIC_ADMISSION_SOLVED_WITH_SLACK",
        bonesaw_core::DynamicAdmissionFlags::SOLVED_WITH_SLACK,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_SOLVER_NOT_ADMITTED",
        bonesaw_core::DynamicAdmissionFlags::SOLVER_NOT_ADMITTED,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_PREVIOUS_PLAN_EXPIRED",
        bonesaw_core::DynamicAdmissionFlags::PREVIOUS_PLAN_EXPIRED,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_PRIMARY_ACTUATOR_LIMIT",
        bonesaw_core::DynamicAdmissionFlags::PRIMARY_ACTUATOR_LIMIT,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_PRIMARY_JOINT_POSITION_LIMIT",
        bonesaw_core::DynamicAdmissionFlags::PRIMARY_JOINT_POSITION_LIMIT,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_CONTINGENCY_ACTUATOR_LIMIT",
        bonesaw_core::DynamicAdmissionFlags::CONTINGENCY_ACTUATOR_LIMIT,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_CONTINGENCY_JOINT_POSITION_LIMIT",
        bonesaw_core::DynamicAdmissionFlags::CONTINGENCY_JOINT_POSITION_LIMIT,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_PRIMARY_COLLISION",
        bonesaw_core::DynamicAdmissionFlags::PRIMARY_COLLISION,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_CONTINGENCY_COLLISION",
        bonesaw_core::DynamicAdmissionFlags::CONTINGENCY_COLLISION,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_PRIMARY_CONTINUOUS_CLEARANCE",
        bonesaw_core::DynamicAdmissionFlags::PRIMARY_CONTINUOUS_CLEARANCE,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_CONTINGENCY_CONTINUOUS_CLEARANCE",
        bonesaw_core::DynamicAdmissionFlags::CONTINGENCY_CONTINUOUS_CLEARANCE,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_COMMAND_TRACKING_CONTINGENCY",
        bonesaw_core::DynamicAdmissionFlags::COMMAND_TRACKING_CONTINGENCY,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_COMMAND_TRACKING_REJECTED",
        bonesaw_core::DynamicAdmissionFlags::COMMAND_TRACKING_REJECTED,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_ROBOT_OBSERVATION_FUTURE",
        bonesaw_core::DynamicAdmissionFlags::ROBOT_OBSERVATION_FUTURE,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_ROBOT_OBSERVATION_STALE",
        bonesaw_core::DynamicAdmissionFlags::ROBOT_OBSERVATION_STALE,
    )?;
    module.add(
        "DYNAMIC_ADMISSION_ROBOT_OBSERVATION_UNCERTAIN",
        bonesaw_core::DynamicAdmissionFlags::ROBOT_OBSERVATION_UNCERTAIN,
    )?;
    module.add_class::<ActuatorResourceSession>()?;
    module.add_class::<ActuatorRealizationSession>()?;
    module.add_class::<RobotObservationHistorySession>()?;
    module.add_class::<CollisionAvoidanceSession>()?;
    module.add_class::<FloatingCollisionBarrierSession>()?;
    module.add_class::<WorldSdfBarrierSession>()?;
    module.add_class::<ControllerSession>()?;
    module.add_class::<KinematicWitnessSession>()?;
    module.add_class::<CpuMirrorBatchSession>()?;
    module.add_class::<ContactTransitionModelSession>()?;
    module.add_class::<UpkieBalanceSession>()?;
    module.add_class::<FloatingWbcSession>()?;
    module.add_class::<DynamicAdvanceSession>()?;
    Ok(())
}
