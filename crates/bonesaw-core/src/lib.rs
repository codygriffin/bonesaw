//! Bonesaw's CPU reference implementation.
//!
//! The crate is intentionally a library: callers own time, state, output, and
//! scratch.  The first implementation slice is kinematic WBC plus deterministic
//! segment synthesis; the public types leave room for the dynamics gate.

pub mod actuation;
pub mod authority;
pub mod collision;
pub mod contact_command_lease;
pub mod contact_observation;
pub mod contact_program_authority;
pub mod contact_transition;
pub mod controller;
pub mod dynamic_controller;
pub mod dynamic_wbc;
pub mod external_load;
pub mod frames;
pub mod history;
pub mod ik;
pub mod lipm;
pub mod math;
pub mod model;
pub mod program;
pub mod rig;
pub mod signal;
pub mod solver;
pub mod support;
pub mod support_contingency;
pub mod terminal_impact;
pub mod trajectory;
pub mod urdf;
pub mod viability_confirmation;
pub mod viability_execution_monitor;
pub mod viability_forecast;
pub mod viability_hybrid_guard;
pub mod viability_poll;
pub mod viability_request;
pub mod world_collision;

pub use actuation::{
    ActuationError, ActuatorId, ActuatorLimits, ActuatorRealizationProfile,
    ActuatorRealizationSample, ActuatorRealizationState, ActuatorResourceModel,
    ActuatorResourceSample, ActuatorResourceState, ActuatorSpec, CompiledActuation,
    DifferentialDriveMap, PassiveActuatorRealizationSample, WheelCommand,
    step_actuator_realization, step_actuator_resource, step_passive_actuator_realization,
};
pub use authority::{
    ActuatorEffortUtilization, AuthorityEvidenceError, JointPositionHeadroom,
    maximum_actuator_effort_utilization, minimum_joint_position_headroom,
};
pub use collision::{
    CollisionAccelerationBarrierConfig, CollisionAccelerationBarrierEvidence,
    CollisionAccelerationBarrierScratch, CollisionAvoidanceConfig, CollisionError,
    CollisionEvaluationScratch, CollisionPair, CollisionSweepReport, CompiledCollisionModel,
    DistanceQuality, DistanceSample, SphereProxy,
};
pub use contact_command_lease::{
    CONTACT_COMMAND_CONTACTLESS, CONTACT_COMMAND_EVIDENCE_REVOKED, CONTACT_COMMAND_EXPIRED,
    CONTACT_COMMAND_FRESH, CONTACT_COMMAND_HARD_SUPPORT_CHANGED, CONTACT_COMMAND_INVALID,
    CONTACT_COMMAND_LEASED, CONTACT_COMMAND_SEQUENCE_REJECTED,
    CONTACT_COMMAND_STABLE_SUPPORT_CHANGED, CONTACT_COMMAND_SUPPORT_REVOKED,
    CONTACT_COMMAND_TRANSITION, ContactCommandLeaseConfig, ContactCommandLeaseOutput,
    ContactCommandLeaseState, ContactCommandLeaseStatus, ContactCommandProvenance,
    step_contact_command_lease,
};
pub use contact_observation::{
    CONTACT_OBSERVATION_DEBOUNCING, CONTACT_OBSERVATION_FUTURE, CONTACT_OBSERVATION_INVALID_CONFIG,
    CONTACT_OBSERVATION_MISSING, CONTACT_OBSERVATION_SEQUENCE, CONTACT_OBSERVATION_SOURCE,
    CONTACT_OBSERVATION_STALE, CONTACT_OBSERVATION_TIMESTAMP, CONTACT_OBSERVATION_TRANSITION,
    CONTACT_OBSERVATION_UNCERTAIN, ContactObservation, ContactObservationConfig,
    ContactObservationOutput, ContactObservationProvenance, ContactObservationState,
    ContactObservationStatus, step_contact_observation,
};
pub use contact_program_authority::{
    ContactProgramAuthorityConfig, ContactProgramAuthorityOutput, ContactProgramAuthorityState,
    ContactProgramSelection, step_contact_program_authority,
    step_contact_program_authority_with_inexact_command,
};
pub use contact_transition::{
    CONTACT_TRANSITION_IMPULSE_WIDTH, CONTACT_TRANSITION_WITNESS_WIDTH,
    CompliantContactImpulseInput, CompliantFrictionCone, CompliantStepIntegrator,
    ContactTransitionAccelerationIntervalInput, ContactTransitionError, ContactTransitionInput,
    ContactTransitionResponseError, ContactTransitionResponseScratch,
    CoupledContactHypothesisEnvelopeInput, CoupledContactImpulseError, CoupledContactImpulseInput,
    CoupledPositiveReferenceCompliantContactImpulseInput,
    DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH, DirectionalContactTransitionInput,
    ModelCoupledPositiveReferenceCompliantContactImpulseInput,
    ModelCoupledPositiveReferenceContactError, ModelCoupledPositiveReferenceContactScratch,
    PointImpulseResponseSpec, PositiveReferenceCompliantContactImpulseInput, SPATIAL_IMPULSE_WIDTH,
    SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH, SpatialImpulseResponseSpec,
    SpatialPatchTransitionInput, solve_coupled_contact_impulse,
    solve_coupled_positive_reference_compliant_contact_impulse,
    solve_model_coupled_positive_reference_compliant_contact_impulse,
    solve_positive_reference_compliant_contact_impulse, solve_substepped_compliant_contact_impulse,
    write_contact_transition_acceleration_interval_bounds, write_contact_transition_bounds,
    write_coupled_contact_hypothesis_velocity_envelope,
    write_directional_contact_transition_bounds, write_generalized_momentum_impulse_residuals,
    write_generalized_velocity_bounds_from_kinetic_impulse_ellipsoid,
    write_generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids,
    write_generalized_velocity_interval_from_momentum_box, write_point_impulse_velocity_response,
    write_point_impulse_velocity_response_with_delassus, write_spatial_impulse_velocity_response,
    write_spatial_patch_contact_transition_bounds,
};
pub use controller::{
    Controller, ControllerConfig, ControllerInput, ControllerOutput, ControllerOutputBuffer,
    ControllerScratch, ControllerState, FrameTarget, StepStatus,
};
pub use dynamic_controller::{
    CollisionContinuityPolicy, CommandTrackingAction, CommandTrackingEvidence,
    CommandTrackingLimits, DynamicAdmissionFlags, DynamicControllerError, DynamicPlanSelection,
    DynamicStepStatus, DynamicTrajectoryValidationConfig, FloatingDynamicController,
    FloatingDynamicControllerInput, FloatingDynamicControllerOutput,
    FloatingDynamicControllerScratch, FloatingDynamicControllerSolvedInput,
    FloatingDynamicControllerState, UnknownCollisionGeometryPolicy,
};
pub use dynamic_wbc::{
    ActuatorEffortInput, BalanceFeedbackAuthorityConfig, BalanceFeedbackAuthorityOutput,
    ContactMode, ContactPhaseAuthorityConfig, ContactPhaseAuthorityOutput, ContactSpec,
    DcmBalanceConfig, DcmBalanceOutput, DynamicWbc, DynamicWbcConfig, DynamicWbcError,
    DynamicWbcInput, DynamicWbcOutput, DynamicWbcScratch, FLOATING_ANGULAR_TASK_CAPACITY,
    FLOATING_POINT_TASK_CAPACITY, FLOATING_TASK_DIAGNOSTIC_CAPACITY, FloatingCenterOfMassTask,
    FloatingCentroidalAngularMomentumTask, FloatingDynamicWbc, FloatingDynamicWbcInput,
    FloatingDynamicWbcOutput, FloatingDynamicWbcScratch, FloatingFrameAngularAccelerationTask,
    FloatingJointAccelerationTask, FloatingPointAccelerationTask, FloatingTaskPriorities,
    FloatingTaskResidual, FloatingTaskWeights, MeasuredContactPhase, SupportPatchSpec,
    balance_feedback_authority, contact_phase_authority, dcm_balance_acceleration,
    joint_acceleration_interval, joint_acceleration_interval_with_observation_error,
    joint_velocity_envelope_acceleration, slew_contact_phase_authority,
};
pub use external_load::{
    DeclaredExternalWrench, ExternalLoadClass, ExternalLoadError, ExternalLoadFrame,
    ExternalLoadProvenance, ExternalLoadSource, validate_declared_external_wrench,
};
pub use frames::{
    AtlasFrameEstimate, AtlasFrameId, CompiledFrameAtlas, DerivedFrameOp, ExternalFrameInputs,
    ExternalFrameSlotId, FrameAtlasError, FrameAtlasSnapshot, HistoricalAtlasEstimate,
    HistoricalFrameQuery, HistoricalFrameQueryError, RootedFrameInput,
};
pub use history::{
    ExternalFrameHistories, ExternalFrameHistory, ExternalFrameSample, HistoryInsert,
    HistoryQueryError, HistoryQueryPolicy, ReconstructedExternalFrame, ReconstructedState,
    ReconstructionProvenance, RobotHistory, RobotObservationErrorBound, RobotObservationErrorError,
    RobotObservationErrorGrowth, RobotObservationEvidence, RobotObservationHistory,
    RobotObservationIngestDisposition, RobotObservationIngestReport, RobotObservationLimits,
    RobotObservationQueryError, RobotObservationQueryPolicy,
    RobotObservationReconstructionEvidence, RobotObservationRef, RobotObservationStamp,
    TimedRobotState, evaluate_robot_observation,
};
pub use ik::{
    PlanarIkError, PlanarIkOptions, PlanarIkReport, PlanarIkScratch, PlanarPointIkTarget,
    WholeBodyIkError, WholeBodyIkOptions, WholeBodyIkReport, WholeBodyIkScratch,
    WholeBodyJetOptions, WholeBodyJetReport, WholeBodyPointIkTarget, WholeBodyPointJetTarget,
    solve_planar_point_ik_into, solve_whole_body_ik_into, solve_whole_body_kinematic_jets_into,
};
pub use lipm::{
    ConvexSupportPolygon, LipmBoundaryPlan, LipmBoundaryPlannerConfig, LipmPlanError, LipmSample,
    LipmState,
};
pub use math::{
    ControlTime, Force6, Motion6, SpatialAcceleration6, SymmetricMat6, Transform3, Vec2, Vec3,
};
pub use model::{
    BodyId, CollisionShape, CompiledModel, DynamicsCache, FloatingRobotState, FrameEstimate,
    FrameId, FrameQuery, JointId, JointKind, ModelCache, RobotState, VisualShape,
};
pub use program::{MotionProgram, ProgramHeader, TimingSpec};
pub use rig::{
    CompiledTaskProgram, FloatingTaskCommand, FloatingTaskState, TaskCompileError,
    TaskEvaluationError, TaskOp, TaskSpec,
};
pub use signal::{
    CompiledSignalProgram, RotationJet, ScalarJet, SignalCompileError, SignalEvaluationError,
    SignalInputFrame, SignalJet, SignalKind, SignalMemory, SignalMemoryCell, SignalOp,
    SignalOutputBuffer, SignalOutputSpec, SignalScratch, VectorJet,
};
pub use solver::{
    ConstraintBuffer, HierarchicalSolver, LinearConstraint, Priority, SolveDiagnostics,
    SolveResult, SolveStatus, SolverWorkspace, Task, TaskBuffer, TaskKind, VelocityBounds,
};
pub use support::{
    CaptureLandingRetargetConfig, CaptureLandingRetargetOutput, SupportPhase,
    SupportTransitionConfig, SupportTransitionError, SupportTransitionObservation,
    SupportTransitionState, TouchdownPhaseRetimingConfig, TouchdownPhaseRetimingInput,
    TouchdownPhaseRetimingOutput, capture_landing_retarget, cubic_precontact_acceleration,
    sample_quintic_vector_jet, slew_touchdown_phase_rate, support_margin_phase_rate,
    time_warp_vector_jet, touchdown_phase_retiming,
};
pub use support_contingency::{
    SupportContingencyConfig, SupportContingencyEvidence, SupportContingencyMode,
    write_support_contingency_request,
};
pub use terminal_impact::{
    ConservativeTerminalImpactDeltaSelection, ConservativeTerminalImpactSelection,
    TERMINAL_IMPACT_PAIRED_COMPONENTS, TerminalImpactCandidate, TerminalImpactComponentDeltaBox,
    TerminalImpactConfig, TerminalImpactError, TerminalImpactPairedStateExemplar,
    TerminalImpactPairedStateTube, TerminalImpactScore, TerminalImpactState,
    TerminalImpactStateBox, TerminalImpactVelocityBoxState,
    bound_terminal_impact_paired_state_delta, score_terminal_impact,
    score_terminal_impact_paired_state_exemplar_delta, score_terminal_impact_state_box_upper,
    score_terminal_impact_velocity_box_upper, select_conservative_terminal_impact_candidate,
    select_conservative_terminal_impact_delta_candidate,
    write_terminal_impact_hypothesis_envelopes,
};
pub use trajectory::{
    ActuatorSample, ActuatorSampleBlock, QuinticSegment, RootPosePredictionSegment,
    RootPredictionErrorGrowth, SegmentLimits, TrajectoryError,
};
pub use urdf::{UrdfError, load_urdf, load_urdf_file};
pub use viability_confirmation::{
    VIABILITY_CONFIRMATION_CONFIRMED, VIABILITY_CONFIRMATION_DIRECTION_RESET,
    VIABILITY_CONFIRMATION_EVIDENCE_REVOKED, VIABILITY_CONFIRMATION_EXPIRED,
    VIABILITY_CONFIRMATION_INPUT_REJECTED, VIABILITY_CONFIRMATION_SEQUENCE_REJECTED,
    VIABILITY_CONFIRMATION_SHADOWING, VIABILITY_CONFIRMATION_SUPPORT_CHANGED,
    VIABILITY_CONFIRMATION_TRANSITION, ViabilityConfirmationConfig, ViabilityConfirmationOutput,
    ViabilityConfirmationState, ViabilityConfirmationStatus, step_viability_confirmation,
};
pub use viability_execution_monitor::{
    VIABILITY_EXECUTION_COMPONENTS, VIABILITY_EXECUTION_COVERED,
    VIABILITY_EXECUTION_EVIDENCE_REJECTED, VIABILITY_EXECUTION_EXCEEDED,
    VIABILITY_EXECUTION_INPUT_REJECTED, VIABILITY_EXECUTION_PREDICTION_UNAVAILABLE,
    VIABILITY_EXECUTION_SEQUENCE_REJECTED, VIABILITY_EXECUTION_SUPPORT_CHANGED,
    VIABILITY_EXECUTION_TRANSITION, VIABILITY_EXECUTION_WARMUP, VIABILITY_EXECUTION_WINDOW,
    ViabilityExecutionMonitorConfig, ViabilityExecutionMonitorOutput,
    ViabilityExecutionMonitorState, ViabilityExecutionMonitorStatus,
    step_viability_execution_monitor,
};
pub use viability_forecast::{
    INEXACT_OBSERVATION_AUTHORITY_LEVELS_Q15, InexactObservationAuthoritySelection,
    VIABILITY_FORECAST_KNOTS, ViabilityForecastCandidate, ViabilityForecastConfig,
    ViabilityForecastError, ViabilityForecastKnot, ViabilityForecastScore, ViabilityForecastState,
    predict_viability_forecast_path, score_viability_forecast,
    select_inexact_observation_authority,
};
pub use viability_hybrid_guard::{
    VIABILITY_HYBRID_GUARD_ADMITTED, VIABILITY_HYBRID_GUARD_DIRECTION_REJECTED,
    VIABILITY_HYBRID_GUARD_EVIDENCE_REJECTED, VIABILITY_HYBRID_GUARD_INPUT_REJECTED,
    VIABILITY_HYBRID_GUARD_LOAD_REJECTED, VIABILITY_HYBRID_GUARD_SEQUENCE_REJECTED,
    VIABILITY_HYBRID_GUARD_SUPPORT_AGE_REJECTED, VIABILITY_HYBRID_GUARD_SUPPORT_CHANGED,
    VIABILITY_HYBRID_GUARD_TRANSITION, ViabilityHybridGuardConfig, ViabilityHybridGuardOutput,
    ViabilityHybridGuardState, ViabilityHybridGuardStatus, step_viability_hybrid_guard,
};
pub use viability_poll::{
    ViabilityPollConfig, ViabilityPollError, ViabilityPollOutput, ViabilityPollState,
    next_viability_poll,
};
pub use viability_request::{
    VIABILITY_REQUEST_ACTIVE, VIABILITY_REQUEST_EVIDENCE_REVOKED, VIABILITY_REQUEST_EXPIRED,
    VIABILITY_REQUEST_FRESH, VIABILITY_REQUEST_HELD, VIABILITY_REQUEST_INPUT_REJECTED,
    VIABILITY_REQUEST_RELEASING, VIABILITY_REQUEST_SEQUENCE_REJECTED,
    VIABILITY_REQUEST_SLEW_LIMITED, VIABILITY_REQUEST_TRANSITION, ViabilityRequestConfig,
    ViabilityRequestOutput, ViabilityRequestProvenance, ViabilityRequestState,
    ViabilityRequestStatus, step_viability_request,
};
pub use world_collision::{
    CompiledWorldCollisionModel, DenseSdfGrid, SdfOutsidePolicy, SdfSample, SdfSampleSource,
    WorldCollisionBarrierEvidence, WorldCollisionBarrierScratch, WorldCollisionContinuityReport,
    WorldCollisionError, WorldCollisionEvaluationScratch, WorldCollisionSweepReport,
    WorldCollisionSweepScratch, WorldDistanceSample, WorldSceneEvidence, WorldSceneStamp,
    WorldSceneValidity, WorldSphereProbe,
};
