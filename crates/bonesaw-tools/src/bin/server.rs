use std::{
    collections::HashMap,
    env,
    net::SocketAddr,
    path::PathBuf,
    process::Stdio,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{Context, Result, anyhow};
use axum::{
    Router,
    extract::{
        State,
        ws::{Message, WebSocket, WebSocketUpgrade},
    },
    http::{HeaderValue, header::CACHE_CONTROL},
    response::IntoResponse,
    routing::get,
};
use bonesaw_core::{
    BodyId, CollisionAccelerationBarrierConfig, CollisionContinuityPolicy, CollisionShape,
    CommandTrackingAction, CommandTrackingLimits, CompiledCollisionModel, CompiledModel,
    CompiledWorldCollisionModel, ContactMode, ContactSpec, Controller, ControllerConfig,
    ControllerInput, ControllerOutputBuffer, ControllerScratch, ControllerState,
    DeclaredExternalWrench, DenseSdfGrid, DistanceQuality, DynamicPlanSelection,
    DynamicTrajectoryValidationConfig, DynamicWbcConfig, DynamicsCache, ExternalLoadClass,
    ExternalLoadError, ExternalLoadFrame, ExternalLoadProvenance, ExternalLoadSource,
    FLOATING_TASK_DIAGNOSTIC_CAPACITY, FloatingDynamicController, FloatingDynamicControllerOutput,
    FloatingDynamicControllerScratch, FloatingDynamicControllerSolvedInput,
    FloatingDynamicControllerState, FloatingDynamicWbc, FloatingDynamicWbcInput,
    FloatingDynamicWbcOutput, FloatingJointAccelerationTask, FloatingRobotState,
    FloatingTaskCommand, FloatingTaskResidual, FloatingTaskState, FrameId, FrameTarget, ModelCache,
    Motion6, MotionProgram, PlanarIkOptions, PlanarIkScratch, PlanarPointIkTarget, Priority,
    ReconstructionProvenance, RobotObservationErrorBound, RobotObservationErrorGrowth,
    RobotObservationHistory, RobotObservationIngestReport, RobotObservationLimits,
    RobotObservationQueryPolicy, RobotObservationReconstructionEvidence, RobotObservationRef,
    RobotObservationStamp, RobotState, RootPosePredictionSegment, RootPredictionErrorGrowth,
    RotationJet, SdfOutsidePolicy, SdfSampleSource, SignalInputFrame, SignalMemory,
    SignalOutputBuffer, SignalScratch, SolveStatus, SpatialAcceleration6, StepStatus,
    SupportPatchSpec, TimingSpec, Vec3, VectorJet, VelocityBounds, WorldSceneStamp,
    WorldSceneValidity, joint_acceleration_interval_with_observation_error,
    maximum_actuator_effort_utilization, minimum_joint_position_headroom,
    solve_planar_point_ik_into, validate_declared_external_wrench,
};
use bonesaw_cuda::{
    ContactKinematicMode, ContactLockSpec, PointQuerySpec, RigidPatchBasisSpec,
    derive_rigid_patch_contact_modes,
};
use bonesaw_tools::{UpkieWheelBalancer, UpkieWheelBalancerState, compile_floating_balance_policy};
use futures_util::{SinkExt, StreamExt};
use nalgebra::{DMatrix, DVector, Point3, Translation3, UnitQuaternion};
use serde::{Deserialize, Serialize};
use tokio::{
    io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
    process::Command,
    time::MissedTickBehavior,
};
use tower_http::{services::ServeDir, set_header::SetResponseHeaderLayer, trace::TraceLayer};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

const LIVE_SELF_COLLISION_CLEARANCE_M: f64 = 0.02;
const LIVE_WORLD_COLLISION_CLEARANCE_M: f64 = 0.02;
const LIVE_WORLD_WALL_Y_M: f64 = 0.45;
const LIVE_WORLD_SCENE_EPOCH: u64 = 115;
const LIVE_COMMAND_TRACKING_LIMITS: CommandTrackingLimits = CommandTrackingLimits {
    position_contingency: Some(0.25),
    position_reject: Some(0.75),
    velocity_contingency: Some(2.0),
    velocity_reject: Some(6.0),
};
const LIVE_ROBOT_OBSERVATION_LIMITS: RobotObservationLimits = RobotObservationLimits {
    maximum_age_ns: Some(25_000_000),
    maximum_synchronization_uncertainty_ns: Some(2_000_000),
};
const LIVE_ROBOT_OBSERVATION_HISTORY_CAPACITY: usize = 64;
const LIVE_PLANT_MAXIMUM_FORCE_N: f64 = 8.0;
const LIVE_PLANT_MAXIMUM_APPLICATION_OFFSET_M: f64 = 0.75;
const LIVE_PLANT_COMMAND_TTL_MS: u64 = 140;
const LIVE_PLANT_WORKER_TIMEOUT_MS: u64 = 100;
/// Keeps the source visual tire mesh above the exact z=0 plane while leaving
/// MuJoCo's physical ground and collision semantics unchanged.
const LIVE_GROUND_REGISTRATION_MARGIN_M: f64 = 0.00025;
const LIVE_STREAM_PERIOD_NS: i64 = 20_000_000;
const LIVE_WBC_DT: f64 = 0.020;
const LIVE_ROBOT_OBSERVATION_QUERY_POLICY: RobotObservationQueryPolicy =
    RobotObservationQueryPolicy {
        maximum_interpolation_gap_ns: 40_000_000,
        maximum_extrapolation_ns: 20_000_000,
        maximum_source_age_ns: 25_000_000,
        maximum_synchronization_uncertainty_ns: 2_000_000,
        allow_prediction: true,
        allow_hold: false,
    };
const LIVE_ROBOT_OBSERVATION_ERROR_GROWTH: RobotObservationErrorGrowth =
    RobotObservationErrorGrowth {
        initial_joint_position_error_rad: 0.0,
        initial_joint_velocity_error_rad_s: 0.0,
        joint_velocity_error_bound_rad_s: 0.08,
        joint_acceleration_error_bound_rad_s2: 6.0,
        initial_root_translation_error_m: 0.0,
        root_linear_velocity_error_bound_m_s: 0.03,
        root_linear_acceleration_error_bound_m_s2: 0.8,
        initial_root_rotation_error_rad: 0.0,
        root_angular_velocity_error_bound_rad_s: 0.04,
        root_angular_acceleration_error_bound_rad_s2: 1.0,
        initial_represented_point_position_error_m: 0.0,
        represented_point_velocity_error_bound_m_s: 0.05,
        represented_point_acceleration_error_bound_m_s2: 1.0,
        initial_center_of_mass_position_error_m: 0.0,
        center_of_mass_velocity_error_bound_m_s: 0.03,
        center_of_mass_acceleration_error_bound_m_s2: 0.5,
    };
const LIVE_ROOT_PREDICTION_ERROR_GROWTH: RootPredictionErrorGrowth = RootPredictionErrorGrowth {
    initial_translation_radius_m: 0.0004,
    translation_velocity_error_bound_mps: 0.020,
    translation_acceleration_error_bound_mps2: 0.50,
    initial_rotation_radius_rad: 0.0003,
    angular_velocity_error_bound_radps: 0.015,
    angular_acceleration_error_bound_radps2: 0.40,
};

#[derive(Clone)]
struct AppState {
    program: Arc<MotionProgram>,
    plant_gateway: Option<Arc<PlantGatewayConfig>>,
}

#[derive(Clone, Debug)]
struct PlantGatewayConfig {
    python: PathBuf,
    worker: PathBuf,
    model: PathBuf,
}

#[derive(Clone, Debug, Serialize)]
struct PlantGatewayContract {
    available: bool,
    websocket_path: Option<&'static str>,
    maximum_force_n: f64,
    maximum_application_offset_m: f64,
    command_ttl_ms: u64,
    worker_timeout_ms: u64,
    plant_owner: &'static str,
    controller_owner: &'static str,
    external_load_protocol: u8,
    accepted_external_load_sources: [&'static str; 2],
    executable_external_load_class: &'static str,
    measured_impact_owner: &'static str,
    unobserved_model_reserve_owner: &'static str,
}

impl PlantGatewayContract {
    fn from_config(config: Option<&PlantGatewayConfig>) -> Self {
        Self {
            available: config.is_some(),
            websocket_path: config.map(|_| "/plant-ws"),
            maximum_force_n: LIVE_PLANT_MAXIMUM_FORCE_N,
            maximum_application_offset_m: LIVE_PLANT_MAXIMUM_APPLICATION_OFFSET_M,
            command_ttl_ms: LIVE_PLANT_COMMAND_TTL_MS,
            worker_timeout_ms: LIVE_PLANT_WORKER_TIMEOUT_MS,
            plant_owner: "python_mujoco",
            controller_owner: "rust_bonesaw",
            external_load_protocol: 2,
            accepted_external_load_sources: ["interactive_operator", "evaluation_harness"],
            executable_external_load_class: "declared_continuous_wrench",
            measured_impact_owner: "python_mujoco_contacts",
            unobserved_model_reserve_owner: "not_estimated_by_live_gateway",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum PlantExternalLoadSource {
    InteractiveOperator,
    EvaluationHarness,
    SupervisorySystem,
    Unavailable,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum PlantExternalLoadClass {
    DeclaredContinuousWrench,
    MeasuredImpactImpulse,
    UnobservedModelReserve,
    Unavailable,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum PlantExternalLoadFrame {
    World,
    Unavailable,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
struct PlantExternalLoadProvenance {
    source: PlantExternalLoadSource,
    load_class: PlantExternalLoadClass,
    force_frame: PlantExternalLoadFrame,
    application_point_frame: PlantExternalLoadFrame,
}

impl PlantExternalLoadProvenance {
    fn core(self) -> ExternalLoadProvenance {
        ExternalLoadProvenance {
            source: match self.source {
                PlantExternalLoadSource::InteractiveOperator => {
                    ExternalLoadSource::InteractiveOperator
                }
                PlantExternalLoadSource::EvaluationHarness => ExternalLoadSource::EvaluationHarness,
                PlantExternalLoadSource::SupervisorySystem => ExternalLoadSource::SupervisorySystem,
                PlantExternalLoadSource::Unavailable => ExternalLoadSource::Unavailable,
            },
            load_class: match self.load_class {
                PlantExternalLoadClass::DeclaredContinuousWrench => {
                    ExternalLoadClass::DeclaredContinuousWrench
                }
                PlantExternalLoadClass::MeasuredImpactImpulse => {
                    ExternalLoadClass::MeasuredImpactImpulse
                }
                PlantExternalLoadClass::UnobservedModelReserve => {
                    ExternalLoadClass::UnobservedModelReserve
                }
                PlantExternalLoadClass::Unavailable => ExternalLoadClass::Unavailable,
            },
            force_frame: match self.force_frame {
                PlantExternalLoadFrame::World => ExternalLoadFrame::World,
                PlantExternalLoadFrame::Unavailable => ExternalLoadFrame::Unavailable,
            },
            application_point_frame: match self.application_point_frame {
                PlantExternalLoadFrame::World => ExternalLoadFrame::World,
                PlantExternalLoadFrame::Unavailable => ExternalLoadFrame::Unavailable,
            },
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ClientCommand {
    Drag {
        frame: String,
        target: [f64; 3],
        request_id: Option<u64>,
    },
    JointDrag {
        frame: String,
        coordinate: usize,
        target_position: f64,
        request_id: Option<u64>,
    },
    QueryFrames {
        request_id: u64,
        joint_positions: Vec<f64>,
    },
    SetObservationTransport {
        mode: ObservationTransportMode,
    },
    Release {
        request_id: Option<u64>,
    },
    Reset,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
enum PlantClientCommand {
    PlantPush {
        body: String,
        force_world: [f64; 3],
        application_point_world: [f64; 3],
        provenance: PlantExternalLoadProvenance,
        request_id: u64,
    },
    PlantRelease {
        request_id: u64,
    },
    /// Stop advancing MuJoCo while keeping the plant stream alive.  Pause is
    /// deliberately a typed plant command instead of an overloaded release:
    /// a release drops the wrench lease, whereas pause freezes measured state
    /// and can be resumed without rebuilding the worker.
    PlantPause {
        request_id: u64,
    },
    /// Resume the worker's 250 Hz physics / 50 Hz WBC loop after a pause.
    PlantResume {
        request_id: u64,
    },
    PlantReset {
        request_id: u64,
    },
}

#[derive(Clone, Debug)]
struct ActivePlantPush {
    body: String,
    force_world: [f64; 3],
    application_point_world: [f64; 3],
    provenance: PlantExternalLoadProvenance,
    request_id: u64,
    expires_at: Instant,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum ObservationTransportMode {
    #[default]
    Exact,
    Interpolated,
    Predicted,
    Stale,
}

impl ObservationTransportMode {
    const INTERPOLATION_LOOKBACK_NS: i64 = 10_000_000;

    fn query_time_ns(self, tick_time_ns: i64) -> i64 {
        match self {
            Self::Interpolated => tick_time_ns.saturating_sub(Self::INTERPOLATION_LOOKBACK_NS),
            Self::Exact | Self::Predicted | Self::Stale => tick_time_ns,
        }
    }

    fn emits_sample(self, sample_time_ns: i64, history_is_empty: bool) -> bool {
        match self {
            Self::Exact | Self::Interpolated => true,
            // A 25 Hz producer makes every other 50 Hz WBC query a genuine
            // 20 ms constant-velocity prediction.
            Self::Predicted => history_is_empty || sample_time_ns.rem_euclid(40_000_000) == 0,
            Self::Stale => false,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Exact => "exact",
            Self::Interpolated => "interpolated",
            Self::Predicted => "predicted",
            Self::Stale => "stale",
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct Bone {
    parent: usize,
    child: usize,
}

#[derive(Clone, Debug, Serialize)]
struct InteractionHandle {
    frame: String,
    kind: &'static str,
    label: String,
}

#[derive(Clone, Debug, Serialize)]
struct JointLimitContract {
    coordinate: usize,
    name: String,
    lower: Option<f64>,
    upper: Option<f64>,
    velocity: f64,
}

#[derive(Clone, Debug, Serialize)]
struct RenderVisual {
    shape: RenderGeometry,
    rgba: [f64; 4],
}

#[derive(Clone, Copy, Debug, Serialize)]
struct RenderSupportPoint {
    body: usize,
    point_in_body: [f64; 3],
}

#[derive(Clone, Debug, Serialize)]
struct RenderSupportPatch {
    stable_id: u32,
    minimum_margin_m: f64,
    points: Vec<RenderSupportPoint>,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct RenderWorldSdfPlane {
    stable_id: u32,
    point: [f64; 3],
    normal: [f64; 3],
    extent_x_m: f64,
    extent_z_m: f64,
    label: &'static str,
}

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum RenderGeometry {
    Sphere {
        body: usize,
        radius: f64,
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
    },
    Capsule {
        body: usize,
        radius: f64,
        half_length: f64,
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
    },
    Box {
        body: usize,
        half_extents: [f64; 3],
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
    },
    Cylinder {
        body: usize,
        radius: f64,
        half_length: f64,
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
    },
    Mesh {
        body: usize,
        filename: String,
        scale: [f64; 3],
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
    },
}

#[derive(Clone, Debug, Serialize)]
struct Metrics {
    authority_profile: &'static str,
    /// Maximum end-to-end query time in this streamed frame. This is the value
    /// compared with the per-query solver budget.
    solve_us: f64,
    /// Sum of end-to-end query times represented by this streamed frame.
    query_batch_us: f64,
    query_count: usize,
    guided_preview_wbc_admitted: Option<bool>,
    interaction_target_clamped: bool,
    interaction_target_clamp_error_m: f64,
    /// Minimum represented collision-shape height above the physical z=0
    /// ground plane. Negative values are penetration.
    minimum_collision_ground_clearance_m: Option<f64>,
    intent_residual: f64,
    trajectory_velocity_scale: Option<f64>,
    trajectory_backtrack_steps: Option<usize>,
    min_bound_margin: f64,
    active_constraint_rows: Vec<u32>,
    dynamics_residual_linf: Option<f64>,
    contact_residual_linf: Option<f64>,
    maximum_constraint_violation: Option<f64>,
    raw_minimum_support_margin_m: Option<f64>,
    minimum_support_margin_m: Option<f64>,
    limiting_support_patch: Option<u32>,
    collision_barrier_minimum_distance_m: Option<f64>,
    raw_collision_barrier_minimum_margin_m: Option<f64>,
    collision_barrier_minimum_margin_m: Option<f64>,
    collision_barrier_closest_pair: Option<u32>,
    collision_barrier_closest_body_a: Option<usize>,
    collision_barrier_closest_body_b: Option<usize>,
    collision_barrier_active_pairs: usize,
    collision_barrier_limiting_pair: Option<u32>,
    collision_barrier_limiting_body_a: Option<usize>,
    collision_barrier_limiting_body_b: Option<usize>,
    collision_barrier_quality: Option<&'static str>,
    collision_barrier_relative_velocity_mps: Option<f64>,
    collision_barrier_required_acceleration_mps2: Option<f64>,
    collision_barrier_achieved_acceleration_mps2: Option<f64>,
    collision_barrier_residual_mps2: Option<f64>,
    collision_barrier_unsupported_shapes: usize,
    world_collision_minimum_distance_m: Option<f64>,
    raw_world_collision_minimum_margin_m: Option<f64>,
    world_collision_minimum_margin_m: Option<f64>,
    world_collision_closest_probe: Option<u32>,
    world_collision_closest_body: Option<usize>,
    world_collision_active_probes: usize,
    world_collision_limiting_probe: Option<u32>,
    world_collision_limiting_body: Option<usize>,
    world_collision_field_source: Option<&'static str>,
    world_collision_proxy_quality: Option<&'static str>,
    world_collision_gradient_norm: Option<f64>,
    world_collision_relative_velocity_mps: Option<f64>,
    world_collision_required_acceleration_mps2: Option<f64>,
    world_collision_achieved_acceleration_mps2: Option<f64>,
    world_collision_residual_mps2: Option<f64>,
    world_collision_unsupported_shapes: usize,
    world_collision_outside_policy: &'static str,
    center_of_mass_world: Option<[f64; 3]>,
    minimum_friction_margin: Option<f64>,
    limiting_actuator: Option<usize>,
    maximum_torque_utilization: Option<f64>,
    limiting_joint: Option<usize>,
    raw_minimum_joint_margin_rad: Option<f64>,
    minimum_joint_margin_rad: Option<f64>,
    minimum_joint_headroom_fraction: Option<f64>,
    joint_position_headroom_rad: Vec<Option<f64>>,
    limiting_joint_stopping: Option<usize>,
    raw_minimum_joint_stopping_margin_rad_s2: Option<f64>,
    minimum_joint_stopping_margin_rad_s2: Option<f64>,
    joint_stopping_recovery_count: usize,
    limiting_joint_velocity_stopping: Option<usize>,
    minimum_joint_velocity_stopping_headroom_rad: Option<f64>,
    joint_velocity_stopping_headroom_rad: Vec<Option<f64>>,
    command_selection: Option<&'static str>,
    command_admission_flags: Option<u32>,
    command_tracking_action: Option<&'static str>,
    command_tracking_position_error: Option<f64>,
    command_tracking_velocity_error: Option<f64>,
    command_tracking_limiting_position_actuator: Option<usize>,
    command_tracking_limiting_velocity_actuator: Option<usize>,
    command_tracking_position_contingency_headroom: Option<f64>,
    command_tracking_position_reject_headroom: Option<f64>,
    command_tracking_velocity_contingency_headroom: Option<f64>,
    command_tracking_velocity_reject_headroom: Option<f64>,
    robot_observation_source_time_ns: Option<i64>,
    robot_observation_mapped_time_ns: Option<i64>,
    robot_observation_source_sequence: Option<u64>,
    robot_observation_source_id: Option<u64>,
    robot_observation_synchronization_uncertainty_ns: Option<i64>,
    robot_observation_age_ns: Option<i64>,
    robot_observation_age_headroom_ns: Option<i64>,
    robot_observation_synchronization_headroom_ns: Option<i64>,
    robot_observation_causal: Option<bool>,
    robot_observation_age_valid: Option<bool>,
    robot_observation_synchronization_valid: Option<bool>,
    robot_observation_history_len: usize,
    robot_observation_history_capacity: usize,
    robot_observation_ingest_accepted: usize,
    robot_observation_ingest_ignored: usize,
    robot_observation_ingest_rejected: usize,
    robot_observation_transport_mode: &'static str,
    robot_observation_transport_query_time_ns: Option<i64>,
    robot_observation_transport_sample_emitted: bool,
    robot_observation_frame_exact_queries: usize,
    robot_observation_frame_interpolated_queries: usize,
    robot_observation_frame_predicted_queries: usize,
    robot_observation_frame_held_queries: usize,
    robot_observation_error_exposure_ns: Option<i64>,
    robot_observation_joint_position_error_rad: Option<f64>,
    robot_observation_joint_velocity_error_rad_s: Option<f64>,
    robot_observation_root_translation_error_m: Option<f64>,
    robot_observation_root_rotation_error_rad: Option<f64>,
    robot_observation_point_position_error_m: Option<f64>,
    robot_observation_center_of_mass_position_error_m: Option<f64>,
    robot_observation_reconstruction_provenance: Option<&'static str>,
    robot_observation_reconstruction_lower_time_ns: Option<i64>,
    robot_observation_reconstruction_upper_time_ns: Option<i64>,
    robot_observation_reconstruction_lower_source_id: Option<u64>,
    robot_observation_reconstruction_upper_source_id: Option<u64>,
    robot_observation_reconstruction_lower_sequence: Option<u64>,
    robot_observation_reconstruction_upper_sequence: Option<u64>,
    robot_observation_reconstruction_source_age_ns: Option<i64>,
    robot_observation_reconstruction_source_age_headroom_ns: Option<i64>,
    robot_observation_reconstruction_synchronization_headroom_ns: Option<i64>,
    robot_observation_reconstruction_hard_eligible: Option<bool>,
    command_clearance_requirement_m: Option<f64>,
    primary_sampled_clearance_m: Option<f64>,
    primary_robust_sampled_clearance_m: Option<f64>,
    primary_continuous_clearance_m: Option<f64>,
    primary_robust_continuous_clearance_m: Option<f64>,
    primary_continuous_limiting_pair: Option<u32>,
    primary_continuous_limiting_body_a: Option<usize>,
    primary_continuous_limiting_body_b: Option<usize>,
    primary_continuous_relative_speed_m_s: Option<f64>,
    primary_continuity_leaf_intervals: usize,
    primary_refinement_pair_samples: usize,
    primary_continuity_unresolved_intervals: usize,
    primary_continuity_maximum_subdivision_depth: u8,
    primary_minimum_collision_pair: Option<u32>,
    primary_minimum_collision_body_a: Option<usize>,
    primary_minimum_collision_body_b: Option<usize>,
    primary_first_collision_pair: Option<u32>,
    primary_first_collision_body_a: Option<usize>,
    primary_first_collision_body_b: Option<usize>,
    primary_first_collision_time_ns: Option<i64>,
    contingency_sampled_clearance_m: Option<f64>,
    contingency_robust_sampled_clearance_m: Option<f64>,
    contingency_continuous_clearance_m: Option<f64>,
    contingency_robust_continuous_clearance_m: Option<f64>,
    contingency_continuous_limiting_pair: Option<u32>,
    contingency_continuous_limiting_body_a: Option<usize>,
    contingency_continuous_limiting_body_b: Option<usize>,
    contingency_continuous_relative_speed_m_s: Option<f64>,
    contingency_continuity_leaf_intervals: usize,
    contingency_refinement_pair_samples: usize,
    contingency_continuity_unresolved_intervals: usize,
    contingency_continuity_maximum_subdivision_depth: u8,
    primary_world_sampled_clearance_m: Option<f64>,
    primary_world_robust_sampled_clearance_m: Option<f64>,
    primary_world_minimum_probe: Option<u32>,
    primary_world_minimum_body: Option<usize>,
    primary_world_field_source: Option<&'static str>,
    primary_world_first_violation_probe: Option<u32>,
    primary_world_first_violation_body: Option<usize>,
    primary_world_first_violation_time_ns: Option<i64>,
    primary_world_first_unknown_probe: Option<u32>,
    primary_world_first_unknown_body: Option<usize>,
    primary_world_first_unknown_time_ns: Option<i64>,
    primary_world_continuous_clearance_m: Option<f64>,
    primary_world_robust_continuous_clearance_m: Option<f64>,
    primary_world_continuous_limiting_probe: Option<u32>,
    primary_world_continuous_limiting_body: Option<usize>,
    primary_world_distance_rate_bound_m_s: Option<f64>,
    primary_world_leaf_intervals: usize,
    primary_world_refinement_probe_samples: usize,
    primary_world_unresolved_intervals: usize,
    primary_world_maximum_subdivision_depth: u8,
    contingency_world_sampled_clearance_m: Option<f64>,
    contingency_world_robust_sampled_clearance_m: Option<f64>,
    contingency_world_minimum_probe: Option<u32>,
    contingency_world_minimum_body: Option<usize>,
    contingency_world_field_source: Option<&'static str>,
    contingency_world_first_violation_probe: Option<u32>,
    contingency_world_first_violation_body: Option<usize>,
    contingency_world_first_violation_time_ns: Option<i64>,
    contingency_world_first_unknown_probe: Option<u32>,
    contingency_world_first_unknown_body: Option<usize>,
    contingency_world_first_unknown_time_ns: Option<i64>,
    contingency_world_continuous_clearance_m: Option<f64>,
    contingency_world_robust_continuous_clearance_m: Option<f64>,
    contingency_world_continuous_limiting_probe: Option<u32>,
    contingency_world_continuous_limiting_body: Option<usize>,
    contingency_world_distance_rate_bound_m_s: Option<f64>,
    contingency_world_leaf_intervals: usize,
    contingency_world_refinement_probe_samples: usize,
    contingency_world_unresolved_intervals: usize,
    contingency_world_maximum_subdivision_depth: u8,
    world_scene_epoch: Option<u64>,
    world_scene_source_time_ns: Option<i64>,
    world_scene_age_ns: Option<i64>,
    world_scene_valid_until_ns: Option<i64>,
    world_scene_horizon_end_ns: Option<i64>,
    world_scene_validity: Option<&'static str>,
    primary_root_prediction_translation_m: Option<f64>,
    primary_root_prediction_rotation_rad: Option<f64>,
    primary_root_prediction_max_linear_speed_m_s: Option<f64>,
    primary_root_prediction_max_angular_speed_rad_s: Option<f64>,
    primary_root_prediction_translation_error_radius_m: Option<f64>,
    primary_root_prediction_rotation_error_radius_rad: Option<f64>,
    primary_world_prediction_clearance_erosion_m: Option<f64>,
    contingency_root_prediction_translation_m: Option<f64>,
    contingency_root_prediction_rotation_rad: Option<f64>,
    contingency_world_prediction_clearance_erosion_m: Option<f64>,
    command_admission_us: Option<f64>,
    command_admission_batch_us: Option<f64>,
    task_pseudoinverse_calls: usize,
    task_jacobi_sweeps: usize,
    clipped_steps: usize,
    feasibility_projection_sweeps: usize,
    feasibility_polish_iterations: usize,
    sample_count: usize,
    clipped_levels: Vec<Priority>,
    task_residuals: [FloatingTaskResidual; FLOATING_TASK_DIAGNOSTIC_CAPACITY],
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum AuthorityAvailability {
    Measured,
    Unavailable,
    Unmodeled,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct AuthorityCapability {
    stable_id: &'static str,
    layer: &'static str,
    signal: &'static str,
    availability: AuthorityAvailability,
    reason: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct AuthorityContract {
    schema: u32,
    source: &'static str,
    signals: Vec<AuthorityCapability>,
}

impl AuthorityContract {
    fn editor(has_finite_support: bool) -> Self {
        use AuthorityAvailability::{Measured, Unavailable, Unmodeled};
        Self {
            schema: 1,
            source: if has_finite_support {
                "bonesaw-tools/flat-foot-editor-r121"
            } else {
                "bonesaw-tools/upkie-editor-r121"
            },
            signals: vec![
                AuthorityCapability {
                    stable_id: "hard_rows",
                    layer: "Invariant",
                    signal: "dynamics/contact/inequality residual",
                    availability: Measured,
                    reason: "computed while the raw floating WBC is active; idle preview emits no hard-row sample",
                },
                AuthorityCapability {
                    stable_id: "finite_support",
                    layer: "Viability",
                    signal: "signed eroded-polygon margin and limiting patch ID",
                    availability: if has_finite_support {
                        Measured
                    } else {
                        Unavailable
                    },
                    reason: if has_finite_support {
                        "computed from the loaded finite-foot force points and exact eroded CoP rows"
                    } else {
                        "Upkie wheel contacts do not define a finite-area support patch"
                    },
                },
                AuthorityCapability {
                    stable_id: "self_collision_avoidance",
                    layer: "Viability",
                    signal: "closest represented pair and limiting acceleration-barrier residual",
                    availability: Measured,
                    reason: "shared tight-primitive closest-feature witness emits a local relative-degree-two hard row while command admission remains separate",
                },
                AuthorityCapability {
                    stable_id: "world_collision_avoidance",
                    layer: "Viability",
                    signal: "closest body sphere/SDF probe and limiting acceleration-barrier residual",
                    availability: Measured,
                    reason: "immutable dense SDF with analytic trilinear gradient and explicit reject/occupied-boundary unknown-space policy",
                },
                AuthorityCapability {
                    stable_id: "world_scene_snapshot",
                    layer: "Scene authority",
                    signal: "scene epoch, source age, validity window, and command-horizon coverage",
                    availability: Measured,
                    reason: "the immutable SDF snapshot is independently versioned in smooth control_world; mismatch, future, stale, or horizon-expired evidence fails closed",
                },
                AuthorityCapability {
                    stable_id: "joint_position",
                    layer: "Resource",
                    signal: "nearest finite position-limit headroom",
                    availability: Measured,
                    reason: "computed from observed coordinates and URDF position limits",
                },
                AuthorityCapability {
                    stable_id: "joint_stopping",
                    layer: "Viability",
                    signal: "next-tick position/velocity/braking acceleration interval",
                    availability: Measured,
                    reason: "derived from observed q/v and authored q/v limits, then intersected with raw-WBC qdd bounds",
                },
                AuthorityCapability {
                    stable_id: "actuator_effort",
                    layer: "Resource",
                    signal: "absolute solved effort / authored effort limit",
                    availability: Measured,
                    reason: "computed from raw-WBC actuator torque and URDF effort limits while dynamic execution is active",
                },
                AuthorityCapability {
                    stable_id: "robot_observation_history",
                    layer: "State reconstruction",
                    signal: "canonical ring ingest, reconstruction provenance, deterministic error growth, raw/robust physical margins, and hard eligibility",
                    availability: Measured,
                    reason: "every live floating-WBC query consumes only a fixed-capacity canonical reconstruction; deterministic error growth erodes joint stopping, support, self-collision, and world-SDF hard margins while preserving raw witnesses; held/invalid/stale evidence fails before hard rows",
                },
                AuthorityCapability {
                    stable_id: "robot_observation_authority",
                    layer: "Observation ingest",
                    signal: "source/mapped timestamps, source identity/sequence, age, synchronization uncertainty, and signed headroom",
                    availability: Measured,
                    reason: "caller-authored monotonic mapping is checked fail-closed; Rust reads no clock and preserves future, stale, and synchronization-uncertain causes independently",
                },
                AuthorityCapability {
                    stable_id: "command_tracking_authority",
                    layer: "Command admission",
                    signal: "observed-versus-commanded position/velocity error and signed envelope headroom",
                    availability: Measured,
                    reason: "exact actuation mapping compares observed state with the commanded splice; warning selects brake and hard divergence rejects both plans without a plant claim",
                },
                AuthorityCapability {
                    stable_id: "command_sampled_geometry",
                    layer: "Command admission",
                    signal: "sampled primary/contingency primitive clearance, minimum pair, and first violating pair/time",
                    availability: Measured,
                    reason: "queried on the exact mapped 20 ms command polynomial at 1 ms including both endpoints while dynamic execution is active",
                },
                AuthorityCapability {
                    stable_id: "command_continuous_clearance",
                    layer: "Command admission",
                    signal: "conservative between-sample primitive-clearance lower bound",
                    availability: Measured,
                    reason: "pair-specific analytic speed bounds with deterministic midpoint refinement of unresolved intervals",
                },
                AuthorityCapability {
                    stable_id: "command_root_prediction",
                    layer: "Command prediction",
                    signal: "primary/contingency floating-root travel and bounded twist",
                    availability: Measured,
                    reason: "explicit local-SE(3) prediction in smooth control_world; collision witness only, not an executable root command or plant rollout",
                },
                AuthorityCapability {
                    stable_id: "command_root_prediction_error",
                    layer: "Command prediction",
                    signal: "deterministic translation/attitude error growth and probe-clearance erosion",
                    availability: Measured,
                    reason: "declared radial position, velocity, and acceleration error bounds robustify every SDF sample and continuous certificate; not covariance or a probability claim",
                },
                AuthorityCapability {
                    stable_id: "command_world_sampled_geometry",
                    layer: "Command admission",
                    signal: "sampled primary/contingency body-probe clearance, source, and violation/unknown witness",
                    availability: Measured,
                    reason: "joint polynomials and their paired floating-root predictions are independently scanned against the immutable world SDF at 1 ms",
                },
                AuthorityCapability {
                    stable_id: "command_world_continuous_clearance",
                    layer: "Command admission",
                    signal: "continuous primary/contingency world-SDF clearance certificate and bounded work",
                    availability: Measured,
                    reason: "field Lipschitz and analytic probe-speed bounds drive deterministic depth-bounded midpoint refinement",
                },
                AuthorityCapability {
                    stable_id: "command_selection",
                    layer: "Command admission",
                    signal: "typed primary/contingency/rejected selection and composable flags",
                    availability: Measured,
                    reason: "parallel policy-/physics-free command query; the browser pose remains an independently labelled guided or raw-WBC visualization",
                },
                AuthorityCapability {
                    stable_id: "actuator_realization",
                    layer: "Resource",
                    signal: "bandwidth/slew-limited realized effort",
                    availability: Unmodeled,
                    reason: "no calibrated effort-bandwidth or slew profile",
                },
                AuthorityCapability {
                    stable_id: "acceleration_realization",
                    layer: "Resource",
                    signal: "realized generalized acceleration consequence",
                    availability: Unavailable,
                    reason: "requires a realized-effort model and measured robot state",
                },
                AuthorityCapability {
                    stable_id: "solver_budget",
                    layer: "Compute",
                    signal: "wall time / fixed 20 ms WBC deadline",
                    availability: Measured,
                    reason: "measured around each integrated WBC block",
                },
                AuthorityCapability {
                    stable_id: "thermal_reliability",
                    layer: "Resource",
                    signal: "persistent calibrated thermal/reliability state",
                    availability: Unmodeled,
                    reason: "no calibrated electrical or thermal model in this program",
                },
                AuthorityCapability {
                    stable_id: "task_residuals",
                    layer: "Lexicographic",
                    signal: "per-task priority, RMS residual, rank, and clipping",
                    availability: Measured,
                    reason: "streamed from fixed-capacity Rust task diagnostics",
                },
            ],
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
struct UpperAuthorityThreshold {
    warning: f64,
    critical: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct LowerAuthorityThreshold {
    warning: f64,
    critical: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct AuthorityThresholdProfile {
    schema: u32,
    source: &'static str,
    hard_residual: UpperAuthorityThreshold,
    support_margin_m: Option<LowerAuthorityThreshold>,
    joint_margin_rad: LowerAuthorityThreshold,
    joint_stopping_margin_rad_s2: LowerAuthorityThreshold,
    actuator_utilization: UpperAuthorityThreshold,
    command_clearance_m: LowerAuthorityThreshold,
    world_collision_margin_m: LowerAuthorityThreshold,
    solver_wall_time_us: UpperAuthorityThreshold,
}

impl AuthorityThresholdProfile {
    fn editor(has_finite_support: bool) -> Self {
        Self {
            schema: 1,
            source: "bonesaw-tools/upkie-editor-r52",
            hard_residual: UpperAuthorityThreshold {
                warning: 1.0e-9,
                critical: 1.0e-8,
            },
            // Rolling-wheel contact has no finite support polygon. Reporting a
            // made-up margin profile here would turn absence into false safety.
            support_margin_m: has_finite_support.then_some(LowerAuthorityThreshold {
                warning: 0.03,
                critical: 0.02,
            }),
            joint_margin_rad: LowerAuthorityThreshold {
                warning: 5.0_f64.to_radians(),
                critical: 0.0,
            },
            joint_stopping_margin_rad_s2: LowerAuthorityThreshold {
                warning: 20.0,
                critical: 0.0,
            },
            actuator_utilization: UpperAuthorityThreshold {
                warning: 0.8,
                critical: 1.0,
            },
            command_clearance_m: LowerAuthorityThreshold {
                warning: 0.03,
                critical: LIVE_SELF_COLLISION_CLEARANCE_M,
            },
            world_collision_margin_m: LowerAuthorityThreshold {
                warning: 0.03,
                critical: LIVE_WORLD_COLLISION_CLEARANCE_M,
            },
            solver_wall_time_us: UpperAuthorityThreshold {
                warning: 16_000.0,
                critical: 20_000.0,
            },
        }
    }
}

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
#[allow(clippy::large_enum_variant)]
enum ServerMessage {
    Hello {
        model: String,
        dof: usize,
        bodies: usize,
        body_names: Vec<String>,
        base_execution: BaseExecutionMode,
        #[serde(rename = "squat_execution")]
        deprecated_squat_execution: BaseExecutionMode,
        bones: Vec<Bone>,
        geometry: Vec<RenderGeometry>,
        visual_geometry: Vec<RenderVisual>,
        world_sdf_planes: Vec<RenderWorldSdfPlane>,
        support_patches: Vec<RenderSupportPatch>,
        frame_names: Vec<String>,
        coordinate_names: Vec<String>,
        joint_limits: Vec<JointLimitContract>,
        actuator_names: Vec<String>,
        actuator_resource_models: Vec<bool>,
        interaction_handles: Vec<InteractionHandle>,
        rooted_frames: Vec<String>,
        plant_gateway: PlantGatewayContract,
        program_fingerprint: String,
        authority_thresholds: AuthorityThresholdProfile,
        authority_contract: AuthorityContract,
    },
    State {
        tick: u64,
        reset_epoch: u64,
        status: bonesaw_core::StepStatus,
        frames: Vec<bonesaw_core::controller::FrameState>,
        joint_positions: Vec<f64>,
        joint_velocities: Vec<f64>,
        commanded_joint_velocity: Option<Vec<f64>>,
        command_id: Option<u64>,
        active_frame: Option<String>,
        metrics: Metrics,
    },
    FrameQuery {
        request_id: u64,
        frames: Vec<bonesaw_core::controller::FrameState>,
    },
    ObservationWithheld {
        tick: u64,
        transport_mode: ObservationTransportMode,
        query_time_ns: i64,
        newest_sample_time_ns: Option<i64>,
        reason: String,
    },
    Error {
        message: String,
    },
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
enum BaseExecutionMode {
    RawDynamic,
    GuidedPreview,
}

#[derive(Clone, Debug)]
struct DragTarget {
    frame_name: String,
    frame: FrameId,
    start_world: Vec3,
    target: Vec3,
}

#[derive(Clone, Debug)]
enum ActiveDrag {
    Point(DragTarget),
    Joint {
        frame_name: String,
        coordinate: usize,
        target_position: f64,
    },
    Base {
        frame_name: String,
    },
}

impl ActiveDrag {
    fn frame_name(&self) -> &str {
        match self {
            Self::Point(target) => &target.frame_name,
            Self::Joint { frame_name, .. } => frame_name,
            Self::Base { frame_name } => frame_name,
        }
    }
}

#[derive(Clone, Debug)]
struct SquatContext {
    frame_name: String,
    frame: FrameId,
    standing_root: Vec3,
    standing_frame: Vec3,
    desired_root: Vec3,
    commanded_root: Vec3,
    target_clamped: bool,
    target_clamp_error_m: f64,
    support_targets: Vec<(FrameId, Vec3)>,
    engaged: bool,
}

struct EditorContactProfile {
    contacts: Vec<ContactSpec>,
    support_patches: Vec<SupportPatchSpec>,
    render_support_patches: Vec<RenderSupportPatch>,
}

impl SquatContext {
    fn compile(model: &CompiledModel, state: &RobotState) -> Option<Self> {
        let (frame_name, frame) = ["torso", "pelvis", "base", "chest"]
            .into_iter()
            .find_map(|name| model.frame_id(name).map(|frame| (name.to_owned(), frame)))?;
        let support_frames = [
            ("left_wheel_center", "right_wheel_center"),
            ("left_contact", "right_contact"),
            ("left_foot", "right_foot"),
        ]
        .into_iter()
        .find_map(|(left, right)| Some((model.frame_id(left)?, model.frame_id(right)?)))?;
        let mut cache = ModelCache::new(model);
        model.forward_kinematics(state, &mut cache).ok()?;
        let standing_frame = cache.world_from_body[frame.0].translation.vector;
        let support_targets = [support_frames.0, support_frames.1]
            .into_iter()
            .map(|support| (support, cache.world_from_body[support.0].translation.vector))
            .collect();
        let standing_root = state.control_world_from_root.translation.vector;
        Some(Self {
            frame_name,
            frame,
            standing_root,
            standing_frame,
            desired_root: standing_root,
            commanded_root: standing_root,
            target_clamped: false,
            target_clamp_error_m: 0.0,
            support_targets,
            engaged: false,
        })
    }

    fn interaction_handle(&self) -> InteractionHandle {
        InteractionHandle {
            frame: self.frame_name.clone(),
            kind: "base",
            label: "TORSO · BASE".to_owned(),
        }
    }

    fn update_target(&mut self, target_frame: Vec3) {
        self.desired_root = self.standing_root + target_frame - self.standing_frame;
        self.target_clamped = false;
        self.target_clamp_error_m = 0.0;
        self.engaged = true;
    }

    fn reset(&mut self) {
        self.desired_root = self.standing_root;
        self.commanded_root = self.standing_root;
        self.target_clamped = false;
        self.target_clamp_error_m = 0.0;
        self.engaged = false;
    }

    fn release(&mut self) {
        self.target_clamped = false;
        self.target_clamp_error_m = 0.0;
        self.engaged = false;
    }
}

fn collect_interaction_handles(
    model: &CompiledModel,
    squat: Option<&SquatContext>,
) -> Vec<InteractionHandle> {
    let mut handles: Vec<_> = squat
        .map(SquatContext::interaction_handle)
        .into_iter()
        .collect();
    for joint in &model.joints {
        if joint.coordinate.is_none() || !body_has_movable_ancestor(model, joint.parent) {
            continue;
        }
        let frame = model.bodies[joint.child.0].name.clone();
        if handles.iter().any(|handle| handle.frame == frame) {
            continue;
        }
        handles.push(InteractionHandle {
            frame,
            kind: "joint",
            label: joint.name.replace('_', " ").to_uppercase(),
        });
    }
    handles
}

fn body_has_movable_ancestor(model: &CompiledModel, mut body: BodyId) -> bool {
    while let Some(parent_joint) = model.bodies[body.0].parent_joint {
        let joint = &model.joints[parent_joint.0];
        if joint.coordinate.is_some() {
            return true;
        }
        body = joint.parent;
    }
    false
}

fn compile_editor_contact_profile(
    model: &CompiledModel,
    squat: Option<&SquatContext>,
    wheel_balancer: Option<&UpkieWheelBalancer>,
    total_weight: f64,
) -> Result<EditorContactProfile> {
    let Some(squat) = squat else {
        return Ok(EditorContactProfile {
            contacts: Vec::new(),
            support_patches: Vec::new(),
            render_support_patches: Vec::new(),
        });
    };
    if let Some(balancer) = wheel_balancer {
        let contacts = squat
            .support_targets
            .iter()
            .enumerate()
            .map(|(index, (frame, _))| {
                ContactSpec::horizontal(
                    index as u32 + 1,
                    *frame,
                    Vec3::zeros(),
                    if index == 0 {
                        balancer.left_contact_mode()
                    } else {
                        balancer.right_contact_mode()
                    },
                    0.8,
                    2.0 * total_weight,
                    0.5 * total_weight,
                )
            })
            .collect();
        return Ok(EditorContactProfile {
            contacts,
            support_patches: Vec::new(),
            render_support_patches: Vec::new(),
        });
    }

    const POINTS: [[f64; 3]; 4] = [
        [-0.04, -0.055, -0.07],
        [0.20, -0.055, -0.07],
        [0.20, 0.055, -0.07],
        [-0.04, 0.055, -0.07],
    ];
    let mut queries = Vec::with_capacity(8);
    let mut locks = Vec::with_capacity(8);
    for (foot, (frame, _)) in squat.support_targets.iter().enumerate() {
        for point in 0..4 {
            let slot = foot * 4 + point;
            queries.push(PointQuerySpec {
                stable_id: 8_101 + slot as u32,
                frame_index: frame.0,
                point_in_frame: POINTS[point],
            });
            locks.push(ContactLockSpec {
                stable_id: 8_301 + slot as u32,
                point_query_stable_id: 8_101 + slot as u32,
            });
        }
    }
    let basis_specs = [
        RigidPatchBasisSpec {
            stable_id: 8_401,
            first_contact_slot: 0,
            contact_count: 4,
        },
        RigidPatchBasisSpec {
            stable_id: 8_402,
            first_contact_slot: 4,
            contact_count: 4,
        },
    ];
    let basis = derive_rigid_patch_contact_modes(&queries, &locks, &basis_specs)?;
    let contacts = queries
        .iter()
        .zip(basis.contact_modes.iter().copied())
        .enumerate()
        .map(|(slot, (query, mode))| {
            let (mode, kinematic_enabled) = match mode {
                ContactKinematicMode::Disabled => (ContactMode::LockedPoint, false),
                ContactKinematicMode::LockedPoint => (ContactMode::LockedPoint, true),
                ContactKinematicMode::NormalPoint => (ContactMode::NormalPoint, true),
                ContactKinematicMode::RollingPoint => (ContactMode::RollingPoint, true),
            };
            let mut contact = ContactSpec::horizontal(
                8_301 + slot as u32,
                FrameId(query.frame_index),
                Vec3::from(query.point_in_frame),
                mode,
                0.8,
                2.0 * total_weight,
                total_weight / 8.0,
            );
            contact.kinematic_enabled = kinematic_enabled;
            contact
        })
        .collect::<Vec<_>>();
    let support_patches = basis_specs
        .iter()
        .map(|patch| SupportPatchSpec {
            stable_id: patch.stable_id,
            first_contact: patch.first_contact_slot,
            contact_count: patch.contact_count,
            minimum_margin_m: 0.02,
            minimum_total_normal_force: 0.0,
        })
        .collect::<Vec<_>>();
    let render_support_patches = support_patches
        .iter()
        .map(|patch| RenderSupportPatch {
            stable_id: patch.stable_id,
            minimum_margin_m: patch.minimum_margin_m,
            points: contacts[patch.first_contact..patch.first_contact + patch.contact_count]
                .iter()
                .map(|contact| RenderSupportPoint {
                    body: contact.frame.0,
                    point_in_body: [
                        contact.point_in_frame.x,
                        contact.point_in_frame.y,
                        contact.point_in_frame.z,
                    ],
                })
                .collect(),
        })
        .collect();
    if basis
        .patches
        .iter()
        .any(|patch| patch.rank != 6 || patch.row_count != 6)
    {
        anyhow::bail!("flat-foot editor basis is not six-row full rank");
    }
    if !model.bodies.iter().any(|body| body.name == "left_foot") {
        anyhow::bail!("flat-foot editor requires named left_foot/right_foot frames");
    }
    Ok(EditorContactProfile {
        contacts,
        support_patches,
        render_support_patches,
    })
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let model_path = env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("models/toy_humanoid.urdf"));
    let program = Arc::new(
        compile_editor_program(&model_path)
            .with_context(|| format!("compiling model {}", model_path.display()))?,
    );
    let plant_gateway = if env::var_os("BONESAW_LIVE_PLANT").is_some() {
        let python = env::var_os("BONESAW_PLANT_PYTHON")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("/tmp/bonesaw-mujoco/bin/python"));
        let worker = env::var_os("BONESAW_PLANT_WORKER")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("python/evals/upkie_live_plant_worker.py"));
        if !python.is_file() {
            anyhow::bail!(
                "live plant Python interpreter is missing: {}",
                python.display()
            );
        }
        if !worker.is_file() {
            anyhow::bail!("live plant worker is missing: {}", worker.display());
        }
        Some(Arc::new(PlantGatewayConfig {
            python,
            worker,
            model: model_path.clone(),
        }))
    } else {
        None
    };
    let state = AppState {
        program: Arc::clone(&program),
        plant_gateway,
    };
    let app = Router::new()
        .route("/ws", get(websocket))
        .route("/plant-ws", get(plant_websocket))
        .nest_service("/model-assets", ServeDir::new("models"))
        .fallback_service(ServeDir::new("web").append_index_html_on_directories(true))
        .layer(SetResponseHeaderLayer::overriding(
            CACHE_CONTROL,
            HeaderValue::from_static("no-store"),
        ))
        .layer(TraceLayer::new_for_http())
        .with_state(state);
    let address: SocketAddr = env::var("BONESAW_BIND")
        .unwrap_or_else(|_| "0.0.0.0:8787".into())
        .parse()
        .context("BONESAW_BIND must be an IP:port socket address")?;
    let listener = tokio::net::TcpListener::bind(address).await?;
    info!(
        "Bonesaw lab ready at http://{} ({} DOF, {} bodies)",
        address,
        program.model.dof,
        program.model.bodies.len()
    );
    axum::serve(listener, app).await?;
    Ok(())
}

fn compile_editor_program(model_path: &PathBuf) -> Result<MotionProgram> {
    let base = MotionProgram::compile_urdf_file(model_path, TimingSpec::default(), 1)?;
    let wheeled_balance = base.model.joint_id("left_wheel").is_some();
    let (signals, tasks) = compile_floating_balance_policy(&base.model, wheeled_balance)?;
    let with_signals = base.with_signals(signals)?;
    Ok(with_signals.with_tasks(tasks)?)
}

async fn websocket(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| run_session(socket, state))
}

async fn plant_websocket(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| run_plant_session(socket, state))
}

fn validate_plant_push(command: &PlantClientCommand) -> Result<()> {
    let PlantClientCommand::PlantPush {
        body,
        force_world,
        application_point_world,
        provenance,
        request_id,
        ..
    } = command
    else {
        return Ok(());
    };
    if body.is_empty() || body.len() > 128 {
        anyhow::bail!("plant push body must contain 1–128 bytes");
    }
    if !matches!(
        provenance.source,
        PlantExternalLoadSource::InteractiveOperator | PlantExternalLoadSource::EvaluationHarness
    ) {
        anyhow::bail!("external-load source is not accepted by the live gateway");
    }
    validate_declared_external_wrench(
        DeclaredExternalWrench {
            request_sequence: *request_id,
            provenance: provenance.core(),
            force: *force_world,
            application_point: *application_point_world,
        },
        LIVE_PLANT_MAXIMUM_FORCE_N,
    )
    .map_err(|error| match error {
        ExternalLoadError::ForceLimit => anyhow!(
            "plant push exceeds the {:.1} N force limit",
            LIVE_PLANT_MAXIMUM_FORCE_N
        ),
        ExternalLoadError::NonFinite => anyhow!("plant push vectors must be finite"),
        ExternalLoadError::MissingSource => {
            anyhow!("declared external load requires a source")
        }
        ExternalLoadError::NonExecutableClass => {
            anyhow!("external-load class is evidence-only and cannot execute")
        }
        ExternalLoadError::UnsupportedFrame => {
            anyhow!("declared external load requires world force and point frames")
        }
        ExternalLoadError::InvalidLimit => anyhow!("server external-load limit is invalid"),
    })?;
    Ok(())
}

fn validate_plant_application_point(
    command: &PlantClientCommand,
    body_positions: &HashMap<String, [f64; 3]>,
) -> Result<()> {
    let PlantClientCommand::PlantPush {
        body,
        application_point_world,
        ..
    } = command
    else {
        return Ok(());
    };
    let body_position = body_positions
        .get(body)
        .ok_or_else(|| anyhow!("plant body is unknown or its first state is not ready: {body}"))?;
    let offset_squared = application_point_world
        .iter()
        .zip(body_position)
        .map(|(point, origin)| (point - origin).powi(2))
        .sum::<f64>();
    if offset_squared
        > LIVE_PLANT_MAXIMUM_APPLICATION_OFFSET_M * LIVE_PLANT_MAXIMUM_APPLICATION_OFFSET_M
            + 1.0e-12
    {
        anyhow::bail!(
            "plant application point exceeds the {:.2} m body-origin offset limit",
            LIVE_PLANT_MAXIMUM_APPLICATION_OFFSET_M
        );
    }
    Ok(())
}

fn update_plant_body_positions(
    response: &serde_json::Value,
    body_positions: &mut HashMap<String, [f64; 3]>,
) {
    if response.get("type").and_then(serde_json::Value::as_str) != Some("plant_state") {
        return;
    }
    let Some(frames) = response.get("frames").and_then(serde_json::Value::as_array) else {
        return;
    };
    body_positions.clear();
    for frame in frames {
        let Some(name) = frame.get("name").and_then(serde_json::Value::as_str) else {
            continue;
        };
        let Some(translation) = frame
            .get("translation")
            .and_then(serde_json::Value::as_array)
        else {
            continue;
        };
        if translation.len() != 3 {
            continue;
        }
        let values = [
            translation[0].as_f64(),
            translation[1].as_f64(),
            translation[2].as_f64(),
        ];
        if let [Some(x), Some(y), Some(z)] = values
            && [x, y, z].iter().all(|value| value.is_finite())
        {
            body_positions.insert(name.to_owned(), [x, y, z]);
        }
    }
}

async fn run_plant_session(socket: WebSocket, app: AppState) {
    let (mut sender, mut receiver) = socket.split();
    let Some(config) = app.plant_gateway else {
        let message = serde_json::json!({
            "type": "plant_unavailable",
            "reason": "live MuJoCo plant gateway is disabled",
        });
        let _ = sender.send(Message::Text(message.to_string().into())).await;
        return;
    };
    let mut child = match Command::new(&config.python)
        .arg(&config.worker)
        .arg(&config.model)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .kill_on_drop(true)
        .spawn()
    {
        Ok(child) => child,
        Err(error) => {
            let message = serde_json::json!({
                "type": "plant_unavailable",
                "reason": format!("cannot start live plant worker: {error}"),
            });
            let _ = sender.send(Message::Text(message.to_string().into())).await;
            return;
        }
    };
    let Some(mut stdin) = child.stdin.take() else {
        return;
    };
    let Some(stdout) = child.stdout.take() else {
        return;
    };
    let mut lines = BufReader::new(stdout).lines();
    let hello = match tokio::time::timeout(Duration::from_secs(5), lines.next_line()).await {
        Ok(Ok(Some(line))) => line,
        Ok(Ok(None)) => {
            "{\"type\":\"plant_unavailable\",\"reason\":\"worker closed during startup\"}"
                .to_owned()
        }
        Ok(Err(error)) => serde_json::json!({
            "type": "plant_unavailable",
            "reason": format!("worker startup read failed: {error}"),
        })
        .to_string(),
        Err(_) => {
            "{\"type\":\"plant_unavailable\",\"reason\":\"worker startup timed out\"}".to_owned()
        }
    };
    if sender.send(Message::Text(hello.into())).await.is_err() {
        let _ = child.kill().await;
        return;
    }

    let mut active_push: Option<ActivePlantPush> = None;
    let mut body_positions = HashMap::new();
    let mut command_id: Option<u64> = None;
    let mut reset_requested = false;
    let mut paused = false;
    let mut ticker = tokio::time::interval(Duration::from_millis(20));
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    'session: loop {
        tokio::select! {
            maybe_message = receiver.next() => {
                match maybe_message {
                    Some(Ok(Message::Text(text))) => {
                        if text.len() > 4096 {
                            let message = serde_json::json!({
                                "type": "plant_error",
                                "message": "plant command exceeds 4096 bytes",
                            });
                            let _ = sender.send(Message::Text(message.to_string().into())).await;
                            continue;
                        }
                        match serde_json::from_str::<PlantClientCommand>(&text)
                            .map_err(anyhow::Error::from)
                            .and_then(|command| {
                                validate_plant_push(&command)?;
                                validate_plant_application_point(&command, &body_positions)?;
                                Ok(command)
                            })
                        {
                            Ok(PlantClientCommand::PlantPush {
                                body,
                                force_world,
                                application_point_world,
                                provenance,
                                request_id,
                            }) => {
                                if paused {
                                    let message = serde_json::json!({
                                        "type": "plant_error",
                                        "message": "plant push is disabled while MuJoCo is paused",
                                    });
                                    let _ = sender.send(Message::Text(message.to_string().into())).await;
                                    continue;
                                }
                                command_id = Some(request_id);
                                active_push = Some(ActivePlantPush {
                                    body,
                                    force_world,
                                    application_point_world,
                                    provenance,
                                    request_id,
                                    expires_at: Instant::now()
                                        + Duration::from_millis(LIVE_PLANT_COMMAND_TTL_MS),
                                });
                            }
                            Ok(PlantClientCommand::PlantRelease { request_id }) => {
                                command_id = Some(request_id);
                                active_push = None;
                            }
                            Ok(PlantClientCommand::PlantPause { request_id }) => {
                                command_id = Some(request_id);
                                // A paused simulation must not retain an
                                // operator wrench that can be applied on the
                                // first resumed tick.
                                active_push = None;
                                paused = true;
                            }
                            Ok(PlantClientCommand::PlantResume { request_id }) => {
                                command_id = Some(request_id);
                                paused = false;
                            }
                            Ok(PlantClientCommand::PlantReset { request_id }) => {
                                command_id = Some(request_id);
                                active_push = None;
                                reset_requested = true;
                            }
                            Err(error) => {
                                let message = serde_json::json!({
                                    "type": "plant_error",
                                    "message": format!("invalid plant command: {error}"),
                                });
                                let _ = sender.send(Message::Text(message.to_string().into())).await;
                            }
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => break 'session,
                    Some(Err(error)) => {
                        warn!("plant websocket receive error: {error}");
                        break 'session;
                    }
                    _ => {}
                }
            }
            _ = ticker.tick() => {
                let now = Instant::now();
                let command_expired = active_push
                    .as_ref()
                    .is_some_and(|push| now >= push.expires_at);
                if command_expired {
                    active_push = None;
                }
                let push = active_push.as_ref().map(|push| serde_json::json!({
                    "active": true,
                    "body": push.body,
                    "force_world": push.force_world,
                    "application_point_world": push.application_point_world,
                    "provenance": push.provenance,
                    "request_id": push.request_id,
                }));
                let request = serde_json::json!({
                    "type": "step",
                    "reset": reset_requested,
                    "command_id": command_id,
                    "command_expired": command_expired,
                    "paused": paused,
                    "external_load": push,
                });
                reset_requested = false;
                let mut encoded = request.to_string();
                encoded.push('\n');
                if stdin.write_all(encoded.as_bytes()).await.is_err()
                    || stdin.flush().await.is_err()
                {
                    let message = serde_json::json!({
                        "type": "plant_unavailable",
                        "reason": "live plant worker input closed",
                    });
                    let _ = sender.send(Message::Text(message.to_string().into())).await;
                    break 'session;
                }
                let response = match tokio::time::timeout(
                    Duration::from_millis(LIVE_PLANT_WORKER_TIMEOUT_MS),
                    lines.next_line(),
                )
                .await
                {
                    Ok(Ok(Some(line))) => line,
                    Ok(Ok(None)) => {
                        let message = serde_json::json!({
                            "type": "plant_unavailable",
                            "reason": "live plant worker output closed",
                        });
                        let _ = sender.send(Message::Text(message.to_string().into())).await;
                        break 'session;
                    }
                    Ok(Err(error)) => {
                        let message = serde_json::json!({
                            "type": "plant_unavailable",
                            "reason": format!("live plant worker read failed: {error}"),
                        });
                        let _ = sender.send(Message::Text(message.to_string().into())).await;
                        break 'session;
                    }
                    Err(_) => {
                        let message = serde_json::json!({
                            "type": "plant_unavailable",
                            "reason": format!(
                                "live plant worker exceeded its {} ms fault-tolerant stream budget",
                                LIVE_PLANT_WORKER_TIMEOUT_MS,
                            ),
                        });
                        let _ = sender.send(Message::Text(message.to_string().into())).await;
                        break 'session;
                    }
                };
                let Ok(parsed_response) = serde_json::from_str::<serde_json::Value>(&response) else {
                    let message = serde_json::json!({
                        "type": "plant_unavailable",
                        "reason": "live plant worker emitted invalid JSON",
                    });
                    let _ = sender.send(Message::Text(message.to_string().into())).await;
                    break 'session;
                };
                update_plant_body_positions(&parsed_response, &mut body_positions);
                if parsed_response.get("type").and_then(serde_json::Value::as_str)
                    == Some("plant_error")
                {
                    active_push = None;
                }
                if sender.send(Message::Text(response.into())).await.is_err() {
                    break 'session;
                }
            }
        }
    }
    let _ = child.kill().await;
}

async fn run_session(socket: WebSocket, app: AppState) {
    let kinematic_controller_config = ControllerConfig {
        control_horizon_ns: app.program.timing.control_horizon_ns,
        sample_period_ns: app.program.timing.sample_period_ns,
        ..Default::default()
    };
    let kinematic_max_acceleration = kinematic_controller_config.max_acceleration;
    let controller = match Controller::new(app.program.model.clone(), kinematic_controller_config) {
        Ok(controller) => controller,
        Err(error) => {
            warn!("controller construction failed: {error}");
            return;
        }
    };
    let mut robot = RobotState::zeros(&app.program.model);
    robot.q = standing_posture(&app.program.model);
    if let Ok(root_lift) = ground_root_lift(&app.program.model, &robot) {
        robot.control_world_from_root.translation.vector.z +=
            root_lift + LIVE_GROUND_REGISTRATION_MARGIN_M;
    }
    let guided_squat = env::var_os("BONESAW_LIVE_GUIDED").is_some();
    let standing_posture = robot.q.clone();
    let mut squat = SquatContext::compile(&app.program.model, &robot);
    let mut floating_state = FloatingRobotState {
        robot: robot.clone(),
        root_twist_world: Default::default(),
    };
    let editor_wbc_config = DynamicWbcConfig {
        acceleration_weight: 8.0,
        contact_force_weight: 0.0,
        actuator_torque_weight: 0.0,
        floating_collision_barrier: Some(CollisionAccelerationBarrierConfig {
            hard_margin: LIVE_SELF_COLLISION_CLEARANCE_M,
            influence_margin: 0.10,
            natural_frequency_rad_s: std::f64::consts::TAU * 2.0,
            damping_ratio: 1.0,
            ..CollisionAccelerationBarrierConfig::default()
        }),
        floating_world_collision_barrier: Some(CollisionAccelerationBarrierConfig {
            hard_margin: LIVE_WORLD_COLLISION_CLEARANCE_M,
            influence_margin: 0.09,
            natural_frequency_rad_s: std::f64::consts::TAU * 2.0,
            damping_ratio: 1.0,
            hard_stable_id_base: 0x7000_0000,
        }),
        ..DynamicWbcConfig::default()
    };
    let world_collision = editor_world_collision(&app.program.model)
        .and_then(|world| {
            world.with_scene_stamp(WorldSceneStamp {
                scene_epoch: LIVE_WORLD_SCENE_EPOCH,
                source_time_ns: 0,
                valid_from_ns: 0,
                valid_until_ns: i64::MAX,
            })
        })
        .expect("the built-in editor world SDF and scene stamp are valid");
    let floating_wbc = FloatingDynamicWbc::new_with_world_collision(
        app.program.model.clone(),
        editor_wbc_config.clone(),
        world_collision.clone(),
    )
    .expect("the built-in floating WBC and world configuration are valid");
    let generalized_dof = app.program.model.dof + 6;
    let mut floating_scratch = floating_wbc.scratch(8, 0);
    let mut floating_output = FloatingDynamicWbcOutput::workspace(
        app.program.model.dof,
        8,
        floating_wbc.maximum_constraint_count(8, 0),
    );
    let mut desired_acceleration = DVector::zeros(generalized_dof);
    let mut floating_signal_inputs = SignalInputFrame::with_full_layout(0, 2, 1);
    let mut floating_signal_memory = SignalMemory::new(&app.program.signals);
    let mut floating_signal_memory_next = SignalMemory::new(&app.program.signals);
    let mut floating_signal_scratch = SignalScratch::new(&app.program.signals);
    let mut floating_signal_output = SignalOutputBuffer::new(&app.program.signals);
    let mut floating_task_command = FloatingTaskCommand::default();
    let mut acceleration_bounds = VelocityBounds {
        lower: DVector::from_element(generalized_dof, -200.0),
        upper: DVector::from_element(generalized_dof, 200.0),
    };
    let mut raw_acceleration_bounds = acceleration_bounds.clone();
    let torque_bounds = match actuator_torque_bounds(&app.program) {
        Ok(bounds) => bounds,
        Err(error) => {
            warn!("editor actuation is unsupported: {error}");
            return;
        }
    };
    let total_weight = app
        .program
        .model
        .bodies
        .iter()
        .map(|body| body.mass)
        .sum::<f64>()
        * 9.81;
    let planar_ik_targets = squat
        .as_ref()
        .and_then(|context| compile_upkie_planar_ik_targets(&app.program.model, context));
    let mut planar_ik_scratch = PlanarIkScratch::new(&app.program.model);
    let mut squat_target = robot.clone();
    let mut physical_cache = ModelCache::new(&app.program.model);
    let mut target_cache = ModelCache::new(&app.program.model);
    let mut center_of_mass_dynamics = DynamicsCache::new(&app.program.model);
    let mut center_of_mass_jacobian = DMatrix::zeros(3, generalized_dof);
    let mut contact_jacobian = DMatrix::zeros(3, generalized_dof);
    let mut generalized_velocity = DVector::zeros(generalized_dof);
    if !guided_squat
        && let (Some(context), Some(targets)) = (squat.as_ref(), planar_ik_targets.as_ref())
    {
        if let Err(error) = prepare_balanced_upkie_squat_target(
            &app.program.model,
            &floating_state.robot,
            &standing_posture,
            context.standing_root,
            targets,
            &context.support_targets,
            &mut squat_target,
            &mut planar_ik_scratch,
            &mut target_cache,
        ) {
            warn!("raw standing-pose balance seed failed: {error}");
            return;
        }
        floating_state.robot.clone_from(&squat_target);
        robot.clone_from(&squat_target);
    }
    let wheel_balancer = if app.program.model.joint_id("left_wheel").is_some() {
        match UpkieWheelBalancer::compile(&app.program.model, &floating_state.robot) {
            Ok(balancer) => Some(balancer),
            Err(error) => {
                warn!("Upkie wheel balancer compilation failed: {error}");
                return;
            }
        }
    } else {
        None
    };
    let mut wheel_balancer_state = UpkieWheelBalancerState::default();
    let mut desired_wheel_accelerations = [0.0; 2];
    let target_ground_position = squat.as_ref().map_or(0.0, |context| {
        0.5 * (context.support_targets[0].1.x + context.support_targets[1].1.x)
    });
    let contact_profile = match compile_editor_contact_profile(
        &app.program.model,
        squat.as_ref(),
        wheel_balancer.as_ref(),
        total_weight,
    ) {
        Ok(profile) => profile,
        Err(error) => {
            warn!("editor contact-profile compilation failed: {error}");
            return;
        }
    };
    let mut contacts = contact_profile.contacts;
    let support_patches = contact_profile.support_patches;
    let render_support_patches = contact_profile.render_support_patches;
    if let Err(error) = app
        .program
        .model
        .forward_kinematics(&floating_state.robot, &mut physical_cache)
    {
        warn!("editor contact target FK failed: {error}");
        return;
    }
    let contact_targets = contacts
        .iter()
        .map(|contact| {
            physical_cache.world_from_body[contact.frame.0]
                .transform_point(&Point3::from(contact.point_in_frame))
                .coords
        })
        .collect::<Vec<_>>();
    let command_controller = match FloatingDynamicController::from_program_with_world_collision(
        &app.program,
        editor_wbc_config,
        DynamicTrajectoryValidationConfig {
            self_collision_clearance: Some(LIVE_SELF_COLLISION_CLEARANCE_M),
            collision_continuity: CollisionContinuityPolicy::ConservativeRateBound,
            collision_max_subdivision_depth: 3,
            world_collision_clearance: Some(LIVE_WORLD_COLLISION_CLEARANCE_M),
            world_collision_continuity: CollisionContinuityPolicy::ConservativeRateBound,
            world_collision_max_subdivision_depth: 3,
            root_prediction_error_growth: LIVE_ROOT_PREDICTION_ERROR_GROWTH,
            expected_world_scene_epoch: Some(LIVE_WORLD_SCENE_EPOCH),
            require_world_scene_horizon_validity: true,
            command_tracking_limits: LIVE_COMMAND_TRACKING_LIMITS,
            robot_observation_limits: LIVE_ROBOT_OBSERVATION_LIMITS,
            ..DynamicTrajectoryValidationConfig::default()
        },
        world_collision.clone(),
    ) {
        Ok(controller) => controller,
        Err(error) => {
            warn!("command-admission controller construction failed: {error}");
            return;
        }
    };
    let command_collision = CompiledCollisionModel::compile(&app.program.model);
    let mut command_state =
        match FloatingDynamicControllerState::for_program(&floating_state.robot, &app.program) {
            Ok(state) => state,
            Err(error) => {
                warn!("command-admission state construction failed: {error}");
                return;
            }
        };
    let mut command_state_next = command_state.clone();
    let mut command_output = match FloatingDynamicControllerOutput::workspace_with_world_collision(
        &app.program,
        contacts.len(),
        &world_collision,
    ) {
        Ok(output) => output,
        Err(error) => {
            warn!("command-admission output construction failed: {error}");
            return;
        }
    };
    // `advance_solved_with_observation_error_into` copies the raw WBC block
    // without reallocating, so
    // its storage must match the upstream solver's declared maximum-contact
    // layout even when this model activates fewer contacts.
    command_output.wbc = FloatingDynamicWbcOutput::workspace(
        app.program.model.dof,
        8,
        floating_wbc.maximum_constraint_count(8, 0),
    );
    let mut command_scratch = FloatingDynamicControllerScratch::for_program_with_world_collision(
        &app.program,
        contacts.len(),
        &world_collision,
    );
    let mut robot_observation_history =
        RobotObservationHistory::new(&app.program.model, LIVE_ROBOT_OBSERVATION_HISTORY_CAPACITY);
    let mut reconstructed_observation = FloatingRobotState::zeros(&app.program.model);
    let mut robot_observation_reconstruction: Option<RobotObservationReconstructionEvidence> = None;
    let mut robot_observation_error = RobotObservationErrorBound::default();
    let mut robot_observation_ingest = RobotObservationIngestReport::default();
    let mut observation_transport_mode = ObservationTransportMode::Exact;
    let mut last_observation_sample_time_ns = None;
    let initial_floating_state = floating_state.clone();
    let mut physical_frames = Vec::with_capacity(app.program.model.bodies.len());
    let mut query_state = floating_state.robot.clone();
    let mut query_cache = ModelCache::new(&app.program.model);
    let mut query_frames = Vec::with_capacity(app.program.model.bodies.len());
    let mut controller_state = ControllerState::new(robot.clone(), 0);
    let mut controller_state_next = controller_state.clone();
    let mut scratch = ControllerScratch::new(&app.program.model, 4);
    let mut output_buffer = ControllerOutputBuffer::with_layout(
        &app.program.model,
        app.program.timing.control_horizon_ns,
        app.program.timing.sample_period_ns,
    );
    let mut drag: Option<ActiveDrag> = None;
    let mut command_id = None;
    let mut tick_index = 0_u64;
    let mut reset_epoch = 0_u64;
    let mut ticker = tokio::time::interval(Duration::from_millis(20));
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let (mut sender, mut receiver) = socket.split();
    let has_finite_support = !support_patches.is_empty();

    let hello = ServerMessage::Hello {
        model: app.program.model.name.clone(),
        dof: app.program.model.dof,
        bodies: app.program.model.bodies.len(),
        body_names: app
            .program
            .model
            .bodies
            .iter()
            .map(|body| body.name.clone())
            .collect(),
        base_execution: if guided_squat {
            BaseExecutionMode::GuidedPreview
        } else {
            BaseExecutionMode::RawDynamic
        },
        deprecated_squat_execution: if guided_squat {
            BaseExecutionMode::GuidedPreview
        } else {
            BaseExecutionMode::RawDynamic
        },
        bones: app
            .program
            .model
            .joints
            .iter()
            .map(|joint| Bone {
                parent: joint.parent.0,
                child: joint.child.0,
            })
            .collect(),
        geometry: collect_render_geometry(&app.program.model),
        visual_geometry: collect_render_visuals(&app.program.model),
        world_sdf_planes: vec![RenderWorldSdfPlane {
            stable_id: 9101,
            point: [0.0, LIVE_WORLD_WALL_Y_M, 0.95],
            normal: [0.0, -1.0, 0.0],
            extent_x_m: 1.6,
            extent_z_m: 1.9,
            label: "WORLD SDF",
        }],
        support_patches: render_support_patches,
        frame_names: app
            .program
            .model
            .bodies
            .iter()
            .map(|body| body.name.clone())
            .collect(),
        coordinate_names: app
            .program
            .model
            .coordinate_names()
            .into_iter()
            .map(str::to_owned)
            .collect(),
        joint_limits: app
            .program
            .model
            .joints
            .iter()
            .filter_map(|joint| {
                let coordinate = joint.coordinate?;
                Some(JointLimitContract {
                    coordinate,
                    name: joint.name.clone(),
                    lower: joint.limit.lower.is_finite().then_some(joint.limit.lower),
                    upper: joint.limit.upper.is_finite().then_some(joint.limit.upper),
                    velocity: joint.limit.velocity,
                })
            })
            .collect(),
        actuator_names: app
            .program
            .actuation
            .actuators
            .iter()
            .map(|actuator| actuator.name.clone())
            .collect(),
        actuator_resource_models: app
            .program
            .actuation
            .actuators
            .iter()
            .map(|actuator| actuator.resource_model.is_some())
            .collect(),
        interaction_handles: collect_interaction_handles(&app.program.model, squat.as_ref()),
        rooted_frames: ["control_world", "odom", "map"]
            .into_iter()
            .map(str::to_owned)
            .collect(),
        plant_gateway: PlantGatewayContract::from_config(app.plant_gateway.as_deref()),
        program_fingerprint: fingerprint_hex(&app.program.header.fingerprint_sha256),
        authority_thresholds: AuthorityThresholdProfile::editor(has_finite_support),
        authority_contract: AuthorityContract::editor(has_finite_support),
    };
    if send_json(&mut sender, &hello).await.is_err() {
        return;
    }

    loop {
        tokio::select! {
            maybe_message = receiver.next() => {
                match maybe_message {
                    Some(Ok(Message::Text(text))) => {
                        match serde_json::from_str::<ClientCommand>(&text) {
                            Ok(ClientCommand::Drag {
                                frame,
                                target,
                                request_id,
                            }) => {
                                if let Some(frame_id) = app.program.model.frame_id(&frame)
                                    && target.iter().all(|value| value.is_finite())
                                {
                                    command_id = request_id;
                                    if let Some(squat) = squat.as_mut()
                                        && frame_id == squat.frame
                                    {
                                        squat.update_target(Vec3::from(target));
                                        drag = Some(ActiveDrag::Base {
                                            frame_name: frame,
                                        });
                                    } else {
                                        drag = Some(ActiveDrag::Point(DragTarget {
                                            frame_name: frame,
                                            frame: frame_id,
                                            start_world: physical_cache.world_from_body
                                                [frame_id.0]
                                                .translation
                                                .vector,
                                            target: Vec3::from(target),
                                        }));
                                    }
                                } else {
                                    let message = ServerMessage::Error {
                                        message: format!("unknown frame or invalid target: {frame}"),
                                    };
                                    let _ = send_json(&mut sender, &message).await;
                                }
                            }
                            Ok(ClientCommand::JointDrag {
                                frame,
                                coordinate,
                                target_position,
                                request_id,
                            }) => {
                                if app.program.model.frame_id(&frame).is_some()
                                    && coordinate < app.program.model.dof
                                    && target_position.is_finite()
                                {
                                    command_id = request_id;
                                    drag = Some(ActiveDrag::Joint {
                                        frame_name: frame,
                                        coordinate,
                                        target_position,
                                    });
                                } else {
                                    let message = ServerMessage::Error {
                                        message: format!(
                                            "invalid joint drag: frame={frame}, coordinate={coordinate}, target={target_position}"
                                        ),
                                    };
                                    let _ = send_json(&mut sender, &message).await;
                                }
                            }
                            Ok(ClientCommand::QueryFrames {
                                request_id,
                                joint_positions,
                            }) => {
                                let query_result = if joint_positions.len()
                                    != app.program.model.dof
                                    || !joint_positions.iter().all(|value| value.is_finite())
                                {
                                    Err(anyhow::anyhow!(
                                        "frame query requires {} finite joint positions",
                                        app.program.model.dof
                                    ))
                                } else {
                                    query_state.clone_from(&floating_state.robot);
                                    query_state
                                        .q
                                        .as_mut_slice()
                                        .copy_from_slice(&joint_positions);
                                    query_state
                                        .validate(&app.program.model)
                                        .map_err(anyhow::Error::from)
                                        .and_then(|()| {
                                            collect_frames(
                                                &app.program.model,
                                                &query_state,
                                                &mut query_cache,
                                                &mut query_frames,
                                            )
                                            .map_err(anyhow::Error::from)
                                        })
                                };
                                match query_result {
                                    Ok(()) => {
                                        let message = ServerMessage::FrameQuery {
                                            request_id,
                                            frames: query_frames.clone(),
                                        };
                                        let _ = send_json(&mut sender, &message).await;
                                    }
                                    Err(error) => {
                                        let message = ServerMessage::Error {
                                            message: format!("invalid frame query: {error}"),
                                        };
                                        let _ = send_json(&mut sender, &message).await;
                                    }
                                }
                            }
                            Ok(ClientCommand::Release { request_id }) => {
                                command_id = request_id;
                                drag = None;
                                if let Some(squat) = squat.as_mut() {
                                    squat.release();
                                }
                            }
                            Ok(ClientCommand::SetObservationTransport { mode }) => {
                                if mode != observation_transport_mode
                                    && mode != ObservationTransportMode::Stale
                                {
                                    robot_observation_history.clear();
                                    robot_observation_reconstruction = None;
                                    robot_observation_error = RobotObservationErrorBound::default();
                                    robot_observation_ingest =
                                        RobotObservationIngestReport::default();
                                    last_observation_sample_time_ns = None;
                                }
                                observation_transport_mode = mode;
                            }
                            Ok(ClientCommand::Reset) => {
                                command_id = None;
                                reset_epoch = reset_epoch.saturating_add(1);
                                floating_state.clone_from(&initial_floating_state);
                                robot_observation_history.clear();
                                robot_observation_reconstruction = None;
                                robot_observation_error = RobotObservationErrorBound::default();
                                robot_observation_ingest =
                                    RobotObservationIngestReport::default();
                                observation_transport_mode = ObservationTransportMode::Exact;
                                last_observation_sample_time_ns = None;
                                robot.clone_from(&floating_state.robot);
                                command_state = match FloatingDynamicControllerState::for_program(
                                    &floating_state.robot,
                                    &app.program,
                                ) {
                                    Ok(state) => state,
                                    Err(error) => {
                                        let message = ServerMessage::Error {
                                            message: format!("command-admission reset failed: {error}"),
                                        };
                                        let _ = send_json(&mut sender, &message).await;
                                        continue;
                                    }
                                };
                                command_state_next = command_state.clone();
                                wheel_balancer_state = UpkieWheelBalancerState::default();
                                // This adapter's point-drag controller is
                                // constructed with `Controller::new`, whose
                                // explicit epoch is zero. Reset must preserve
                                // that boundary rather than switching the
                                // state to MotionProgram epoch and causing all
                                // subsequent point drags to be rejected.
                                controller_state = ControllerState::new(robot.clone(), 0);
                                controller_state_next = controller_state.clone();
                                output_buffer = ControllerOutputBuffer::with_layout(
                                    &app.program.model,
                                    app.program.timing.control_horizon_ns,
                                    app.program.timing.sample_period_ns,
                                );
                                floating_signal_memory =
                                    SignalMemory::new(&app.program.signals);
                                floating_signal_memory_next =
                                    SignalMemory::new(&app.program.signals);
                                if let Some(squat) = squat.as_mut() {
                                    squat.reset();
                                }
                                drag = None;
                            }
                            Err(error) => {
                                let message = ServerMessage::Error {
                                    message: format!("invalid command: {error}"),
                                };
                                let _ = send_json(&mut sender, &message).await;
                            }
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Err(error)) => {
                        warn!("websocket receive error: {error}");
                        break;
                    }
                    _ => {}
                }
            }
            _ = ticker.tick() => {
                if let Some(squat) = squat.as_mut()
                    && squat.engaged
                {
                    let mut step_error = None;
                    let mut observation_withheld_query_time_ns = None;
                    let mut joint_stopping_recovery_count = 0;
                    let mut joint_stopping_authority = None;
                    let mut raw_joint_stopping_authority = None;
                    let mut query_batch_us = 0.0;
                    let mut maximum_query_us: f64 = 0.0;
                    let mut command_admission_batch_us = 0.0;
                    let mut maximum_command_admission_us: f64 = 0.0;
                    let mut query_count = 0;
                    let mut reconstruction_query_counts = [0_usize; 4];
                    let mut transport_query_time_ns = None;
                    let mut transport_sample_emitted = false;
                    let mut guided_preview_wbc_admitted = true;
                    for substep in 0..1 {
                        let query_start = Instant::now();
                        let command_tick_time_ns =
                            tick_index as i64 * LIVE_STREAM_PERIOD_NS;
                        transport_query_time_ns = Some(
                            observation_transport_mode.query_time_ns(command_tick_time_ns),
                        );
                        match ingest_live_observation(
                            &mut robot_observation_history,
                            &app.program.model,
                            app.program.header.program_epoch,
                            command_tick_time_ns,
                            tick_index + substep as u64 + 1,
                            &floating_state,
                            observation_transport_mode,
                            &mut robot_observation_ingest,
                        ) {
                            Ok(emitted) => {
                                transport_sample_emitted = emitted;
                                if emitted {
                                    last_observation_sample_time_ns = Some(command_tick_time_ns);
                                }
                            }
                            Err(error) => {
                                step_error = Some(error);
                                observation_withheld_query_time_ns = transport_query_time_ns;
                                break;
                            }
                        }
                        robot_observation_reconstruction =
                            match robot_observation_history.reconstruct_into(
                                &app.program.model,
                                transport_query_time_ns
                                    .expect("transport query time is assigned before reconstruction"),
                                LIVE_ROBOT_OBSERVATION_QUERY_POLICY,
                                &mut reconstructed_observation,
                            ) {
                                Ok(evidence) if evidence.hard_constraint_eligible => {
                                    robot_observation_error = match evidence
                                        .conservative_error_bound(
                                            LIVE_ROBOT_OBSERVATION_ERROR_GROWTH,
                                        )
                                    {
                                        Ok(bound) => bound,
                                        Err(error) => {
                                            step_error = Some(format!(
                                                "robot observation error bound withheld WBC input: {error}"
                                            ));
                                            observation_withheld_query_time_ns =
                                                transport_query_time_ns;
                                            break;
                                        }
                                    };
                                    record_reconstruction_query(
                                        &mut reconstruction_query_counts,
                                        evidence.provenance,
                                    );
                                    Some(evidence)
                                }
                                Ok(evidence) => {
                                    step_error = Some(format!(
                                        "robot observation reconstruction is not hard-eligible: {:?}",
                                        evidence.provenance
                                    ));
                                    observation_withheld_query_time_ns = transport_query_time_ns;
                                    break;
                                }
                                Err(error) => {
                                    step_error = Some(format!(
                                        "robot observation reconstruction withheld WBC input: {error}"
                                    ));
                                    observation_withheld_query_time_ns = transport_query_time_ns;
                                    break;
                                }
                            };
                        let observed_state = &reconstructed_observation;
                        let prior_root_translation = observed_state
                            .robot
                            .control_world_from_root
                            .translation
                            .vector;
                        for axis in 0..3 {
                            squat.commanded_root[axis] += (squat.desired_root[axis]
                                - squat.commanded_root[axis])
                                .clamp(-0.010, 0.010);
                        }
                        let target_root = squat.commanded_root;
                        let target_result = if let Some(targets) = planar_ik_targets.as_ref() {
                            prepare_ground_safe_balanced_upkie_squat_target(
                                &app.program.model,
                                &observed_state.robot,
                                &standing_posture,
                                squat.standing_root,
                                target_root,
                                targets,
                                &squat.support_targets,
                                &mut squat_target,
                                &mut planar_ik_scratch,
                                &mut target_cache,
                            )
                            .map(|(applied_root, clamp_error_m)| {
                                squat.target_clamped = clamp_error_m > 1.0e-6;
                                squat.target_clamp_error_m = clamp_error_m;
                                squat_target.control_world_from_root.translation.vector =
                                    applied_root;
                            })
                        } else {
                            squat.target_clamped = false;
                            squat.target_clamp_error_m = 0.0;
                            squat_target.clone_from(&observed_state.robot);
                            squat_target.control_world_from_root.rotation = Default::default();
                            let planar_drag = drag
                                .as_ref()
                                .and_then(|active| match active {
                                    ActiveDrag::Point(target) => {
                                        Some(target.target - target.start_world)
                                    }
                                    ActiveDrag::Joint { .. } => None,
                                    ActiveDrag::Base { .. } => None,
                                })
                                .unwrap_or_else(Vec3::zeros);
                            squat_target.control_world_from_root.translation.vector.x =
                                planar_drag.x.clamp(-0.16, 0.16);
                            squat_target.control_world_from_root.translation.vector.y =
                                planar_drag.y.clamp(-0.10, 0.10);
                            squat_target.control_world_from_root.translation.vector.z =
                                target_root.z;
                            authored_squat_posture(
                                &app.program.model,
                                squat,
                                target_root.z,
                                &standing_posture,
                                &mut squat_target.q,
                            );
                            app.program
                                .model
                                .forward_kinematics(&squat_target, &mut target_cache)
                                .map_err(anyhow::Error::from)
                        };
                        if let Err(error) = target_result {
                            step_error = Some(error.to_string());
                            break;
                        }
                        for index in 0..app.program.model.dof {
                            squat_target.v[index] =
                                (squat_target.q[index] - observed_state.robot.q[index]) / LIVE_WBC_DT;
                        }
                        let mut squat_target_root_twist = Motion6::default();
                        for axis in 0..3 {
                            squat_target_root_twist.0[3 + axis] =
                                (squat_target.control_world_from_root.translation.vector[axis]
                                    - prior_root_translation[axis])
                                    / LIVE_WBC_DT;
                        }
                        if let Err(error) = stabilize_contacts(
                            &app.program.model,
                            observed_state,
                            &mut contacts,
                            &contact_targets,
                            &mut physical_cache,
                            &mut contact_jacobian,
                            &mut generalized_velocity,
                        ) {
                            step_error = Some(error.to_string());
                            break;
                        }
                        if let Err(error) = app.program.model.floating_com_jacobian_into(
                            &physical_cache,
                            &mut center_of_mass_dynamics,
                            &mut center_of_mass_jacobian,
                        ) {
                            step_error = Some(error.to_string());
                            break;
                        }
                        let mut center_of_mass_velocity = Vec3::zeros();
                        for row in 0..3 {
                            for column in 0..generalized_dof {
                                center_of_mass_velocity[row] += center_of_mass_jacobian
                                    [(row, column)]
                                    * generalized_velocity[column];
                            }
                        }
                        if let Some(balancer) = wheel_balancer.as_ref() {
                            let mut ground_position = 0.0;
                            let mut ground_height = 0.0;
                            for contact in &contacts {
                                let point = physical_cache.world_from_body[contact.frame.0]
                                    .transform_point(&Point3::from(contact.point_in_frame));
                                ground_position += point.x;
                                ground_height += point.z;
                            }
                            ground_position /= contacts.len() as f64;
                            ground_height /= contacts.len() as f64;
                            let virtual_pitch =
                                (physical_cache.center_of_mass_world.x - ground_position).atan2(
                                    (physical_cache.center_of_mass_world.z - ground_height)
                                        .max(0.05),
                                );
                            balancer.emit_accelerations(
                                LIVE_WBC_DT,
                                target_ground_position,
                                ground_position,
                                virtual_pitch,
                                observed_state.robot.v.as_slice(),
                                &mut wheel_balancer_state,
                                &mut desired_wheel_accelerations,
                            );
                        }
                        floating_signal_inputs.rotations[0] = RotationJet {
                            value: Default::default(),
                            ..Default::default()
                        };
                        floating_signal_inputs.vectors[0] = VectorJet {
                            value: squat_target
                                .control_world_from_root
                                .translation
                                .vector,
                            ..Default::default()
                        };
                        floating_signal_inputs.vectors[1] = VectorJet {
                            value: target_cache.center_of_mass_world,
                            ..Default::default()
                        };
                        if let Err(error) = app.program.signals.evaluate_into(
                            LIVE_WBC_DT,
                            &floating_signal_inputs,
                            &floating_signal_memory,
                            &mut floating_signal_memory_next,
                            &mut floating_signal_output,
                            &mut floating_signal_scratch,
                        ) {
                            step_error = Some(error.to_string());
                            break;
                        }
                        if let Err(error) = app.program.tasks.emit_floating_command_into(
                            &floating_signal_output,
                            FloatingTaskState {
                                control_world_from_root: observed_state
                                    .robot
                                    .control_world_from_root,
                                root_twist_world: observed_state.root_twist_world,
                                center_of_mass_world: physical_cache.center_of_mass_world,
                                center_of_mass_velocity_world: center_of_mass_velocity,
                            },
                            &mut floating_task_command,
                        ) {
                            step_error = Some(error.to_string());
                            break;
                        }
                        desired_acceleration.fill(0.0);
                        desired_acceleration.as_mut_slice()[..6]
                            .copy_from_slice(floating_task_command.root_acceleration_world.0.as_slice());
                        prepare_joint_posture_acceleration(
                            observed_state,
                            &squat_target,
                            &mut desired_acceleration,
                        );
                        if let Some(balancer) = wheel_balancer.as_ref() {
                            for row in 0..balancer.coordinates.len() {
                                desired_acceleration[6 + balancer.coordinates[row]] =
                                    desired_wheel_accelerations[row];
                            }
                        }
                        let mut task_priorities = floating_task_command.task_priorities;
                        let mut task_weights = floating_task_command.task_weights;
                        if wheel_balancer.is_some() {
                            task_priorities.joint_posture = Priority::Intent;
                            task_weights.joint_posture = 4.0;
                        }
                        if let Err(error) = compose_joint_stopping_bounds(
                            &app.program.model,
                            &observed_state.robot,
                            0.0,
                            0.0,
                            200.0,
                            LIVE_WBC_DT,
                            &mut raw_acceleration_bounds,
                        ) {
                            step_error = Some(error.to_string());
                            break;
                        }
                        joint_stopping_recovery_count = match compose_joint_stopping_bounds(
                            &app.program.model,
                            &observed_state.robot,
                            robot_observation_error.joint_position_error_rad,
                            robot_observation_error.joint_velocity_error_rad_s,
                            200.0,
                            LIVE_WBC_DT,
                            &mut acceleration_bounds,
                        ) {
                            Ok(count) => count,
                            Err(error) => {
                                step_error = Some(error.to_string());
                                break;
                            }
                        };
                        let input = FloatingDynamicWbcInput {
                            state: &observed_state.robot,
                            root_twist_world: observed_state.root_twist_world,
                            desired_generalized_acceleration: &desired_acceleration,
                            task_priorities,
                            task_weights,
                            joint_posture_weight: 1.0,
                            joint_acceleration_task: wheel_balancer.as_ref().map(|balancer| {
                                FloatingJointAccelerationTask {
                                    coordinates: &balancer.coordinates,
                                    desired_accelerations: &desired_wheel_accelerations,
                                    priority: Priority::Viability,
                                    weight: 1.0,
                                }
                            }),
                            center_of_mass_task: floating_task_command.center_of_mass_task,
                            centroidal_angular_momentum_task: None,
                            frame_angular_acceleration_tasks: &[],
                            point_acceleration_tasks: &[],
                            generalized_acceleration_bounds: &acceleration_bounds,
                            torque_bounds: &torque_bounds,
                            actuator_effort: None,
                            contacts: &contacts,
                            support_patches: &support_patches,
                        };
                        if let Err(error) = floating_wbc.solve_into_with_observation_error(
                            input,
                            robot_observation_error,
                            &mut floating_output,
                            &mut floating_scratch,
                        ) {
                            step_error = Some(error.to_string());
                            break;
                        }
                        let command_admission_start = Instant::now();
                        if let Err(error) = command_controller
                            .advance_solved_with_observation_error_into(
                            FloatingDynamicControllerSolvedInput {
                                tick_time_ns: command_tick_time_ns,
                                program_epoch: app.program.header.program_epoch,
                                observation_stamp: robot_observation_reconstruction
                                    .expect("hard-eligible reconstruction is retained")
                                    .reconstructed_stamp(),
                                observed: &observed_state.robot,
                                root_twist_world: observed_state.root_twist_world,
                                solved_wbc: &floating_output,
                            },
                            robot_observation_error,
                            &command_state,
                            &mut command_state_next,
                            &mut command_output,
                            &mut command_scratch,
                        ) {
                            step_error = Some(format!("command admission failed: {error}"));
                            break;
                        }
                        let command_admission_us =
                            command_admission_start.elapsed().as_secs_f64() * 1e6;
                        command_admission_batch_us += command_admission_us;
                        maximum_command_admission_us =
                            maximum_command_admission_us.max(command_admission_us);
                        std::mem::swap(&mut command_state, &mut command_state_next);
                        std::mem::swap(
                            &mut floating_signal_memory,
                            &mut floating_signal_memory_next,
                        );
                        if !matches!(
                            floating_output.status,
                            SolveStatus::Solved | SolveStatus::SolvedWithSlack
                        ) {
                            if guided_squat {
                                guided_preview_wbc_admitted = false;
                                // The guided editor is an explicit kinematic
                                // preview plus a state-local WBC query. Keep
                                // showing the requested pose when the query is
                                // infeasible so the user can see the red
                                // authority result; never present the preview
                                // as integrated plant response.
                                floating_state.robot.clone_from(&squat_target);
                                floating_state.root_twist_world = squat_target_root_twist;
                                let query_us = (query_start.elapsed().as_secs_f64() * 1e6
                                    - command_admission_us)
                                    .max(0.0);
                                query_batch_us += query_us;
                                maximum_query_us = maximum_query_us.max(query_us);
                                query_count += 1;
                                continue;
                            }
                            step_error = Some(format!(
                                "raw floating WBC returned {:?} (violation={:.3e}, projections={}, polish={}, world_active={}, world_distance={:.6}, world_required={:.6})",
                                floating_output.status,
                                floating_output.solve.maximum_constraint_violation,
                                floating_output.solve.feasibility_halfspace_projections,
                                floating_output.solve.feasibility_polish_iterations,
                                floating_output.world_collision_barrier.active_probe_count,
                                floating_output
                                    .world_collision_barrier
                                    .minimum_signed_distance_m,
                                floating_output
                                    .world_collision_barrier
                                    .limiting_required_normal_acceleration_mps2,
                            ));
                            break;
                        }
                        joint_stopping_authority = minimum_joint_stopping_margin(
                            &floating_output.generalized_acceleration,
                            &acceleration_bounds,
                            app.program.model.dof,
                        );
                        raw_joint_stopping_authority = minimum_joint_stopping_margin(
                            &floating_output.generalized_acceleration,
                            &raw_acceleration_bounds,
                            app.program.model.dof,
                        );
                        if guided_squat {
                            floating_state.robot.clone_from(&squat_target);
                            floating_state.root_twist_world = squat_target_root_twist;
                        } else if let Err(error) = app.program.model.integrate_floating(
                            &mut floating_state,
                            &floating_output.generalized_acceleration,
                            LIVE_WBC_DT,
                        ) {
                            step_error = Some(error.to_string());
                            break;
                        }
                        let query_us = (query_start.elapsed().as_secs_f64() * 1e6
                            - command_admission_us)
                            .max(0.0);
                        query_batch_us += query_us;
                        maximum_query_us = maximum_query_us.max(query_us);
                        query_count += 1;
                    }
                    if let Some(message) = step_error {
                        let message = if let Some(query_time_ns) =
                            observation_withheld_query_time_ns
                        {
                            ServerMessage::ObservationWithheld {
                                tick: tick_index,
                                transport_mode: observation_transport_mode,
                                query_time_ns,
                                newest_sample_time_ns: last_observation_sample_time_ns,
                                reason: message,
                            }
                        } else {
                            ServerMessage::Error { message }
                        };
                        if send_json(&mut sender, &message).await.is_err() {
                            break;
                        }
                        tick_index += 1;
                        continue;
                    }
                    robot.clone_from(&reconstructed_observation.robot);
                    controller_state.observed_state.clone_from(&robot);
                    controller_state.commanded_state.clone_from(&robot);
                    if let Err(error) = collect_frames(
                        &app.program.model,
                        &robot,
                        &mut physical_cache,
                        &mut physical_frames,
                    ) {
                        let message = ServerMessage::Error {
                            message: error.to_string(),
                        };
                        if send_json(&mut sender, &message).await.is_err() {
                            break;
                        }
                        continue;
                    }
                    let status = match floating_output.status {
                        SolveStatus::Solved => StepStatus::Ok,
                        SolveStatus::SolvedWithSlack => StepStatus::Degraded,
                        SolveStatus::MaxIterations
                        | SolveStatus::PrimalInfeasible
                        | SolveStatus::NumericalFailure
                        | SolveStatus::InvalidProblem => StepStatus::Contingency,
                    };
                    let intent_residual = floating_output
                        .solve
                        .level_residuals
                        .iter()
                        .find(|level| level.priority == Priority::Intent)
                        .map_or(0.0, |level| level.l2);
                    let actuator_authority = maximum_actuator_effort_utilization(
                        &floating_output.actuator_torque,
                        &torque_bounds,
                    )
                    .ok()
                    .flatten();
                    let joint_authority = minimum_joint_position_headroom(
                        &app.program.model,
                        &reconstructed_observation.robot,
                    )
                    .ok()
                    .flatten();
                    let primary_collision_bodies = command_output
                        .primary_collision
                        .first_violation_pair_id
                        .and_then(|pair| command_collision.pair_bodies(pair));
                    let primary_minimum_collision_bodies = command_output
                        .primary_collision
                        .minimum_pair_id
                        .and_then(|pair| command_collision.pair_bodies(pair));
                    let primary_continuous_limiting_bodies = command_output
                        .primary_continuous_limiting_pair_id
                        .and_then(|pair| command_collision.pair_bodies(pair));
                    let contingency_continuous_limiting_bodies = command_output
                        .contingency_continuous_limiting_pair_id
                        .and_then(|pair| command_collision.pair_bodies(pair));
                    let collision_barrier_closest_bodies = floating_output
                        .collision_barrier
                        .closest_pair_id
                        .and_then(|pair| command_collision.pair_bodies(pair));
                    let collision_barrier_limiting_bodies = floating_output
                        .collision_barrier
                        .limiting_active_pair_id
                        .and_then(|pair| command_collision.pair_bodies(pair));
                    let world_collision = floating_output.world_collision_barrier;
                    let primary_root_prediction =
                        root_prediction_summary(&command_output.primary_root_prediction);
                    let contingency_root_prediction =
                        root_prediction_summary(&command_output.contingency_root_prediction);
                    let observation_reconstruction = robot_observation_reconstruction
                        .expect("an engaged frame has a hard-eligible boundary reconstruction");
                    let minimum_collision_ground_clearance_m =
                        ground_root_lift(&app.program.model, &reconstructed_observation.robot)
                            .ok()
                            .map(|lift| -lift);
                    let message = ServerMessage::State {
                        tick: tick_index,
                        reset_epoch,
                        status,
                        frames: physical_frames.clone(),
                        joint_positions: reconstructed_observation.robot.q.as_slice().to_vec(),
                        joint_velocities: reconstructed_observation.robot.v.as_slice().to_vec(),
                        commanded_joint_velocity: None,
                        command_id,
                        active_frame: drag
                            .as_ref()
                            .map(|target| target.frame_name().to_owned()),
                        metrics: Metrics {
                            authority_profile: "floating_dynamic_wbc_with_command_admission",
                            solve_us: maximum_query_us,
                            query_batch_us,
                            query_count,
                            guided_preview_wbc_admitted: guided_squat
                                .then_some(guided_preview_wbc_admitted),
                            interaction_target_clamped: squat.target_clamped,
                            interaction_target_clamp_error_m: squat.target_clamp_error_m,
                            minimum_collision_ground_clearance_m,
                            intent_residual,
                            trajectory_velocity_scale: None,
                            trajectory_backtrack_steps: None,
                            min_bound_margin: floating_output.solve.minimum_bound_margin,
                            active_constraint_rows: floating_output
                                .solve
                                .active_constraints
                                .clone(),
                            dynamics_residual_linf: finite_metric(
                                floating_output.dynamics_residual_linf,
                            ),
                            contact_residual_linf: finite_metric(
                                floating_output.contact_acceleration_residual_linf,
                            ),
                            maximum_constraint_violation: finite_metric(
                                floating_output.solve.maximum_constraint_violation,
                            ),
                            raw_minimum_support_margin_m: finite_metric(
                                floating_output.minimum_support_margin_m
                                    + robot_observation_error.center_of_mass_position_error_m,
                            ),
                            minimum_support_margin_m: finite_metric(
                                floating_output.minimum_support_margin_m,
                            ),
                            limiting_support_patch: floating_output.limiting_support_patch,
                            collision_barrier_minimum_distance_m: finite_metric(
                                floating_output
                                    .collision_barrier
                                    .minimum_signed_distance_m,
                            ),
                            raw_collision_barrier_minimum_margin_m: finite_metric(
                                floating_output.collision_barrier.minimum_margin_m
                                    + 2.0
                                        * robot_observation_error
                                            .represented_point_position_error_m,
                            ),
                            collision_barrier_minimum_margin_m: finite_metric(
                                floating_output.collision_barrier.minimum_margin_m,
                            ),
                            collision_barrier_closest_pair: floating_output
                                .collision_barrier
                                .closest_pair_id,
                            collision_barrier_closest_body_a:
                                collision_barrier_closest_bodies.map(|bodies| bodies.0.0),
                            collision_barrier_closest_body_b:
                                collision_barrier_closest_bodies.map(|bodies| bodies.1.0),
                            collision_barrier_active_pairs: floating_output
                                .collision_barrier
                                .active_pair_count,
                            collision_barrier_limiting_pair: floating_output
                                .collision_barrier
                                .limiting_active_pair_id,
                            collision_barrier_limiting_body_a:
                                collision_barrier_limiting_bodies.map(|bodies| bodies.0.0),
                            collision_barrier_limiting_body_b:
                                collision_barrier_limiting_bodies.map(|bodies| bodies.1.0),
                            collision_barrier_quality: floating_output
                                .collision_barrier
                                .limiting_active_quality
                                .or(floating_output.collision_barrier.closest_quality)
                                .map(distance_quality_label),
                            collision_barrier_relative_velocity_mps: floating_output
                                .collision_barrier
                                .limiting_active_pair_id
                                .and_then(|_| {
                                    finite_metric(
                                        floating_output
                                            .collision_barrier
                                            .limiting_relative_normal_velocity_mps,
                                    )
                                }),
                            collision_barrier_required_acceleration_mps2: floating_output
                                .collision_barrier
                                .limiting_active_pair_id
                                .and_then(|_| {
                                    finite_metric(
                                        floating_output
                                            .collision_barrier
                                            .limiting_required_normal_acceleration_mps2,
                                    )
                                }),
                            collision_barrier_achieved_acceleration_mps2: floating_output
                                .collision_barrier
                                .limiting_active_pair_id
                                .and_then(|_| {
                                    finite_metric(
                                        floating_output
                                            .collision_barrier
                                            .limiting_achieved_normal_acceleration_mps2,
                                    )
                                }),
                            collision_barrier_residual_mps2: finite_metric(
                                floating_output
                                    .collision_barrier
                                    .minimum_barrier_residual_mps2,
                            ),
                            collision_barrier_unsupported_shapes: floating_output
                                .collision_barrier
                                .unsupported_shape_count,
                            world_collision_minimum_distance_m: finite_metric(
                                world_collision.minimum_signed_distance_m,
                            ),
                            raw_world_collision_minimum_margin_m: finite_metric(
                                world_collision.minimum_margin_m
                                    + robot_observation_error
                                        .represented_point_position_error_m,
                            ),
                            world_collision_minimum_margin_m: finite_metric(
                                world_collision.minimum_margin_m,
                            ),
                            world_collision_closest_probe: world_collision.closest_probe_id,
                            world_collision_closest_body: world_collision
                                .closest_body
                                .map(|body| body.0),
                            world_collision_active_probes: world_collision.active_probe_count,
                            world_collision_limiting_probe: world_collision
                                .limiting_active_probe_id,
                            world_collision_limiting_body: world_collision
                                .limiting_active_body
                                .map(|body| body.0),
                            world_collision_field_source: world_collision
                                .limiting_field_source
                                .or(world_collision.closest_field_source)
                                .map(sdf_sample_source_label),
                            world_collision_proxy_quality: world_collision
                                .limiting_proxy_quality
                                .or(world_collision.closest_proxy_quality)
                                .map(distance_quality_label),
                            world_collision_gradient_norm: finite_metric(
                                if world_collision.limiting_active_probe_id.is_some() {
                                    world_collision.limiting_gradient_norm
                                } else {
                                    world_collision.closest_gradient_norm
                                },
                            ),
                            world_collision_relative_velocity_mps: world_collision
                                .limiting_active_probe_id
                                .and_then(|_| {
                                    finite_metric(
                                        world_collision.limiting_relative_normal_velocity_mps,
                                    )
                                }),
                            world_collision_required_acceleration_mps2: world_collision
                                .limiting_active_probe_id
                                .and_then(|_| {
                                    finite_metric(
                                        world_collision
                                            .limiting_required_normal_acceleration_mps2,
                                    )
                                }),
                            world_collision_achieved_acceleration_mps2: world_collision
                                .limiting_active_probe_id
                                .and_then(|_| {
                                    finite_metric(
                                        world_collision
                                            .limiting_achieved_normal_acceleration_mps2,
                                    )
                                }),
                            world_collision_residual_mps2: finite_metric(
                                world_collision.minimum_barrier_residual_mps2,
                            ),
                            world_collision_unsupported_shapes: world_collision
                                .unsupported_shape_count,
                            world_collision_outside_policy: match world_collision.outside_policy {
                                SdfOutsidePolicy::Reject => "reject",
                                SdfOutsidePolicy::OccupiedBoundary => "occupied_boundary",
                            },
                            center_of_mass_world: Some([
                                physical_cache.center_of_mass_world.x,
                                physical_cache.center_of_mass_world.y,
                                physical_cache.center_of_mass_world.z,
                            ]),
                            minimum_friction_margin: finite_metric(
                                floating_output.minimum_friction_margin,
                            ),
                            limiting_actuator: actuator_authority
                                .map(|authority| authority.coordinate),
                            maximum_torque_utilization: actuator_authority
                                .map(|authority| authority.utilization),
                            limiting_joint: joint_authority.map(|authority| authority.coordinate),
                            raw_minimum_joint_margin_rad: joint_authority
                                .map(|authority| authority.margin_rad),
                            minimum_joint_margin_rad: joint_authority
                                .map(|authority| {
                                    authority.margin_rad
                                        - robot_observation_error.joint_position_error_rad
                                }),
                            minimum_joint_headroom_fraction: joint_authority.map(|authority| {
                                if authority.margin_rad > 0.0 {
                                    authority.fraction_of_range
                                        * (authority.margin_rad
                                            - robot_observation_error.joint_position_error_rad)
                                        / authority.margin_rad
                                } else {
                                    authority.fraction_of_range
                                }
                            }),
                            joint_position_headroom_rad: joint_position_headrooms(
                                &app.program.model,
                                &reconstructed_observation.robot,
                            ),
                            limiting_joint_stopping: joint_stopping_authority
                                .map(|authority| authority.0),
                            raw_minimum_joint_stopping_margin_rad_s2:
                                raw_joint_stopping_authority.map(|authority| authority.1),
                            minimum_joint_stopping_margin_rad_s2: joint_stopping_authority
                                .map(|authority| authority.1),
                            joint_stopping_recovery_count,
                            limiting_joint_velocity_stopping: None,
                            minimum_joint_velocity_stopping_headroom_rad: None,
                            joint_velocity_stopping_headroom_rad: vec![
                                None;
                                app.program.model.dof
                            ],
                            command_selection: Some(dynamic_selection_label(
                                command_output.selection,
                            )),
                            command_admission_flags: Some(
                                command_output.admission_flags.bits(),
                            ),
                            command_tracking_action: Some(command_tracking_action_label(
                                command_output.command_tracking.action,
                            )),
                            command_tracking_position_error: Some(
                                command_output.command_tracking.maximum_position_error,
                            ),
                            command_tracking_velocity_error: Some(
                                command_output.command_tracking.maximum_velocity_error,
                            ),
                            command_tracking_limiting_position_actuator: command_output
                                .command_tracking
                                .limiting_position_actuator,
                            command_tracking_limiting_velocity_actuator: command_output
                                .command_tracking
                                .limiting_velocity_actuator,
                            command_tracking_position_contingency_headroom: finite_metric(
                                command_output
                                    .command_tracking
                                    .position_contingency_headroom,
                            ),
                            command_tracking_position_reject_headroom: finite_metric(
                                command_output.command_tracking.position_reject_headroom,
                            ),
                            command_tracking_velocity_contingency_headroom: finite_metric(
                                command_output
                                    .command_tracking
                                    .velocity_contingency_headroom,
                            ),
                            command_tracking_velocity_reject_headroom: finite_metric(
                                command_output.command_tracking.velocity_reject_headroom,
                            ),
                            robot_observation_source_time_ns: Some(
                                command_output.robot_observation.stamp.source_time_ns,
                            ),
                            robot_observation_mapped_time_ns: Some(
                                command_output.robot_observation.stamp.mapped_time_ns,
                            ),
                            robot_observation_source_sequence: Some(
                                command_output.robot_observation.stamp.source_sequence,
                            ),
                            robot_observation_source_id: Some(
                                command_output.robot_observation.stamp.source_id,
                            ),
                            robot_observation_synchronization_uncertainty_ns: Some(
                                command_output
                                    .robot_observation
                                    .stamp
                                    .synchronization_uncertainty_ns,
                            ),
                            robot_observation_age_ns: Some(
                                command_output.robot_observation.age_ns,
                            ),
                            robot_observation_age_headroom_ns: Some(
                                command_output.robot_observation.age_headroom_ns,
                            ),
                            robot_observation_synchronization_headroom_ns: Some(
                                command_output
                                    .robot_observation
                                    .synchronization_uncertainty_headroom_ns,
                            ),
                            robot_observation_causal: Some(
                                command_output.robot_observation.causal,
                            ),
                            robot_observation_age_valid: Some(
                                command_output.robot_observation.age_valid,
                            ),
                            robot_observation_synchronization_valid: Some(
                                command_output.robot_observation.synchronization_valid,
                            ),
                            robot_observation_history_len: robot_observation_history.len(),
                            robot_observation_history_capacity: robot_observation_history
                                .capacity(),
                            robot_observation_ingest_accepted: robot_observation_ingest.accepted(),
                            robot_observation_ingest_ignored: observation_ingest_ignored(
                                robot_observation_ingest,
                            ),
                            robot_observation_ingest_rejected: robot_observation_ingest.rejected(),
                            robot_observation_transport_mode: observation_transport_mode.label(),
                            robot_observation_transport_query_time_ns: transport_query_time_ns,
                            robot_observation_transport_sample_emitted: transport_sample_emitted,
                            robot_observation_frame_exact_queries: reconstruction_query_counts[0],
                            robot_observation_frame_interpolated_queries:
                                reconstruction_query_counts[1],
                            robot_observation_frame_predicted_queries:
                                reconstruction_query_counts[2],
                            robot_observation_frame_held_queries: reconstruction_query_counts[3],
                            robot_observation_error_exposure_ns: Some(
                                command_output.observation_error.exposure_ns,
                            ),
                            robot_observation_joint_position_error_rad: Some(
                                command_output.observation_error.joint_position_error_rad,
                            ),
                            robot_observation_joint_velocity_error_rad_s: Some(
                                command_output.observation_error.joint_velocity_error_rad_s,
                            ),
                            robot_observation_root_translation_error_m: Some(
                                command_output.observation_error.root_translation_error_m,
                            ),
                            robot_observation_root_rotation_error_rad: Some(
                                command_output.observation_error.root_rotation_error_rad,
                            ),
                            robot_observation_point_position_error_m: Some(
                                command_output
                                    .observation_error
                                    .represented_point_position_error_m,
                            ),
                            robot_observation_center_of_mass_position_error_m: Some(
                                command_output
                                    .observation_error
                                    .center_of_mass_position_error_m,
                            ),
                            robot_observation_reconstruction_provenance: Some(
                                reconstruction_provenance_label(
                                    observation_reconstruction.provenance,
                                ),
                            ),
                            robot_observation_reconstruction_lower_time_ns: Some(
                                observation_reconstruction.source_interval_ns.0,
                            ),
                            robot_observation_reconstruction_upper_time_ns: Some(
                                observation_reconstruction.source_interval_ns.1,
                            ),
                            robot_observation_reconstruction_lower_source_id: Some(
                                observation_reconstruction.lower_stamp.source_id,
                            ),
                            robot_observation_reconstruction_upper_source_id: Some(
                                observation_reconstruction.upper_stamp.source_id,
                            ),
                            robot_observation_reconstruction_lower_sequence: Some(
                                observation_reconstruction.lower_stamp.source_sequence,
                            ),
                            robot_observation_reconstruction_upper_sequence: Some(
                                observation_reconstruction.upper_stamp.source_sequence,
                            ),
                            robot_observation_reconstruction_source_age_ns: Some(
                                observation_reconstruction.source_age_ns,
                            ),
                            robot_observation_reconstruction_source_age_headroom_ns: Some(
                                observation_reconstruction.source_age_headroom_ns,
                            ),
                            robot_observation_reconstruction_synchronization_headroom_ns: Some(
                                observation_reconstruction.synchronization_headroom_ns,
                            ),
                            robot_observation_reconstruction_hard_eligible: Some(
                                observation_reconstruction.hard_constraint_eligible,
                            ),
                            command_clearance_requirement_m: Some(
                                LIVE_SELF_COLLISION_CLEARANCE_M,
                            ),
                            primary_sampled_clearance_m: finite_metric(
                                command_output.primary_collision.minimum_signed_distance,
                            ),
                            primary_robust_sampled_clearance_m: finite_metric(
                                command_output.primary_robust_collision_minimum_clearance_m,
                            ),
                            primary_continuous_clearance_m: finite_metric(
                                command_output.primary_continuous_clearance_lower_bound,
                            ),
                            primary_robust_continuous_clearance_m: finite_metric(
                                command_output
                                    .primary_robust_continuous_clearance_lower_bound_m,
                            ),
                            primary_continuous_limiting_pair: command_output
                                .primary_continuous_limiting_pair_id,
                            primary_continuous_limiting_body_a:
                                primary_continuous_limiting_bodies.map(|bodies| bodies.0.0),
                            primary_continuous_limiting_body_b:
                                primary_continuous_limiting_bodies.map(|bodies| bodies.1.0),
                            primary_continuous_relative_speed_m_s: finite_metric(
                                command_output.primary_maximum_relative_speed_bound,
                            ),
                            primary_continuity_leaf_intervals: command_output
                                .primary_continuity_leaf_interval_count,
                            primary_refinement_pair_samples: command_output
                                .primary_collision_refinement_pair_samples,
                            primary_continuity_unresolved_intervals: command_output
                                .primary_continuity_unresolved_interval_count,
                            primary_continuity_maximum_subdivision_depth: command_output
                                .primary_continuity_maximum_subdivision_depth,
                            primary_minimum_collision_pair: command_output
                                .primary_collision
                                .minimum_pair_id,
                            primary_minimum_collision_body_a: primary_minimum_collision_bodies
                                .map(|bodies| bodies.0.0),
                            primary_minimum_collision_body_b: primary_minimum_collision_bodies
                                .map(|bodies| bodies.1.0),
                            primary_first_collision_pair: command_output
                                .primary_collision
                                .first_violation_pair_id,
                            primary_first_collision_body_a: primary_collision_bodies
                                .map(|bodies| bodies.0.0),
                            primary_first_collision_body_b: primary_collision_bodies
                                .map(|bodies| bodies.1.0),
                            primary_first_collision_time_ns: command_output
                                .primary_collision
                                .first_violation_time_ns
                                .map(|time_ns| {
                                    time_ns - command_output.primary_segment.start_time_ns
                                }),
                            contingency_sampled_clearance_m: finite_metric(
                                command_output
                                    .contingency_collision
                                    .minimum_signed_distance,
                            ),
                            contingency_robust_sampled_clearance_m: finite_metric(
                                command_output
                                    .contingency_robust_collision_minimum_clearance_m,
                            ),
                            contingency_continuous_clearance_m: finite_metric(
                                command_output.contingency_continuous_clearance_lower_bound,
                            ),
                            contingency_robust_continuous_clearance_m: finite_metric(
                                command_output
                                    .contingency_robust_continuous_clearance_lower_bound_m,
                            ),
                            contingency_continuous_limiting_pair: command_output
                                .contingency_continuous_limiting_pair_id,
                            contingency_continuous_limiting_body_a:
                                contingency_continuous_limiting_bodies.map(|bodies| bodies.0.0),
                            contingency_continuous_limiting_body_b:
                                contingency_continuous_limiting_bodies.map(|bodies| bodies.1.0),
                            contingency_continuous_relative_speed_m_s: finite_metric(
                                command_output.contingency_maximum_relative_speed_bound,
                            ),
                            contingency_continuity_leaf_intervals: command_output
                                .contingency_continuity_leaf_interval_count,
                            contingency_refinement_pair_samples: command_output
                                .contingency_collision_refinement_pair_samples,
                            contingency_continuity_unresolved_intervals: command_output
                                .contingency_continuity_unresolved_interval_count,
                            contingency_continuity_maximum_subdivision_depth: command_output
                                .contingency_continuity_maximum_subdivision_depth,
                            primary_world_sampled_clearance_m: finite_metric(
                                command_output
                                    .primary_world_collision
                                    .minimum_signed_distance_m,
                            ),
                            primary_world_robust_sampled_clearance_m: finite_metric(
                                command_output.primary_robust_world_minimum_clearance_m,
                            ),
                            primary_world_minimum_probe: command_output
                                .primary_world_collision
                                .minimum_probe_id,
                            primary_world_minimum_body: command_output
                                .primary_world_collision
                                .minimum_body
                                .map(|body| body.0),
                            primary_world_field_source: command_output
                                .primary_world_collision
                                .minimum_field_source
                                .map(sdf_sample_source_label),
                            primary_world_first_violation_probe: command_output
                                .primary_world_collision
                                .first_violation_probe_id,
                            primary_world_first_violation_body: command_output
                                .primary_world_collision
                                .first_violation_body
                                .map(|body| body.0),
                            primary_world_first_violation_time_ns: command_output
                                .primary_world_collision
                                .first_violation_time_ns
                                .map(|time_ns| {
                                    time_ns - command_output.primary_segment.start_time_ns
                                }),
                            primary_world_first_unknown_probe: command_output
                                .primary_world_collision
                                .first_unknown_probe_id,
                            primary_world_first_unknown_body: command_output
                                .primary_world_collision
                                .first_unknown_body
                                .map(|body| body.0),
                            primary_world_first_unknown_time_ns: command_output
                                .primary_world_collision
                                .first_unknown_time_ns
                                .map(|time_ns| {
                                    time_ns - command_output.primary_segment.start_time_ns
                                }),
                            primary_world_continuous_clearance_m: finite_metric(
                                command_output
                                    .primary_world_continuity
                                    .minimum_clearance_lower_bound_m,
                            ),
                            primary_world_robust_continuous_clearance_m: finite_metric(
                                command_output
                                    .primary_robust_world_continuous_clearance_lower_bound_m,
                            ),
                            primary_world_continuous_limiting_probe: command_output
                                .primary_world_continuity
                                .limiting_probe_id,
                            primary_world_continuous_limiting_body: command_output
                                .primary_world_continuity
                                .limiting_body
                                .map(|body| body.0),
                            primary_world_distance_rate_bound_m_s: finite_metric(
                                command_output
                                    .primary_world_continuity
                                    .limiting_distance_rate_bound_mps,
                            ),
                            primary_world_leaf_intervals: command_output
                                .primary_world_continuity
                                .leaf_interval_count,
                            primary_world_refinement_probe_samples: command_output
                                .primary_world_continuity
                                .refinement_probe_samples_evaluated,
                            primary_world_unresolved_intervals: command_output
                                .primary_world_continuity
                                .unresolved_interval_count,
                            primary_world_maximum_subdivision_depth: command_output
                                .primary_world_continuity
                                .maximum_subdivision_depth_reached,
                            contingency_world_sampled_clearance_m: finite_metric(
                                command_output
                                    .contingency_world_collision
                                    .minimum_signed_distance_m,
                            ),
                            contingency_world_robust_sampled_clearance_m: finite_metric(
                                command_output.contingency_robust_world_minimum_clearance_m,
                            ),
                            contingency_world_minimum_probe: command_output
                                .contingency_world_collision
                                .minimum_probe_id,
                            contingency_world_minimum_body: command_output
                                .contingency_world_collision
                                .minimum_body
                                .map(|body| body.0),
                            contingency_world_field_source: command_output
                                .contingency_world_collision
                                .minimum_field_source
                                .map(sdf_sample_source_label),
                            contingency_world_first_violation_probe: command_output
                                .contingency_world_collision
                                .first_violation_probe_id,
                            contingency_world_first_violation_body: command_output
                                .contingency_world_collision
                                .first_violation_body
                                .map(|body| body.0),
                            contingency_world_first_violation_time_ns: command_output
                                .contingency_world_collision
                                .first_violation_time_ns
                                .map(|time_ns| {
                                    time_ns - command_output.contingency_segment.start_time_ns
                                }),
                            contingency_world_first_unknown_probe: command_output
                                .contingency_world_collision
                                .first_unknown_probe_id,
                            contingency_world_first_unknown_body: command_output
                                .contingency_world_collision
                                .first_unknown_body
                                .map(|body| body.0),
                            contingency_world_first_unknown_time_ns: command_output
                                .contingency_world_collision
                                .first_unknown_time_ns
                                .map(|time_ns| {
                                    time_ns - command_output.contingency_segment.start_time_ns
                                }),
                            contingency_world_continuous_clearance_m: finite_metric(
                                command_output
                                    .contingency_world_continuity
                                    .minimum_clearance_lower_bound_m,
                            ),
                            contingency_world_robust_continuous_clearance_m: finite_metric(
                                command_output
                                    .contingency_robust_world_continuous_clearance_lower_bound_m,
                            ),
                            contingency_world_continuous_limiting_probe: command_output
                                .contingency_world_continuity
                                .limiting_probe_id,
                            contingency_world_continuous_limiting_body: command_output
                                .contingency_world_continuity
                                .limiting_body
                                .map(|body| body.0),
                            contingency_world_distance_rate_bound_m_s: finite_metric(
                                command_output
                                    .contingency_world_continuity
                                    .limiting_distance_rate_bound_mps,
                            ),
                            contingency_world_leaf_intervals: command_output
                                .contingency_world_continuity
                                .leaf_interval_count,
                            contingency_world_refinement_probe_samples: command_output
                                .contingency_world_continuity
                                .refinement_probe_samples_evaluated,
                            contingency_world_unresolved_intervals: command_output
                                .contingency_world_continuity
                                .unresolved_interval_count,
                            contingency_world_maximum_subdivision_depth: command_output
                                .contingency_world_continuity
                                .maximum_subdivision_depth_reached,
                            world_scene_epoch: Some(command_output.world_scene.stamp.scene_epoch),
                            world_scene_source_time_ns: Some(
                                command_output.world_scene.stamp.source_time_ns,
                            ),
                            world_scene_age_ns: command_output.world_scene.age_ns,
                            world_scene_valid_until_ns: Some(
                                command_output.world_scene.stamp.valid_until_ns,
                            ),
                            world_scene_horizon_end_ns: Some(
                                command_output.world_scene.horizon_end_ns,
                            ),
                            world_scene_validity: Some(world_scene_validity_label(
                                command_output.world_scene.validity,
                            )),
                            primary_root_prediction_translation_m: primary_root_prediction
                                .map(|summary| summary[0]),
                            primary_root_prediction_rotation_rad: primary_root_prediction
                                .map(|summary| summary[1]),
                            primary_root_prediction_max_linear_speed_m_s: primary_root_prediction
                                .map(|summary| summary[2]),
                            primary_root_prediction_max_angular_speed_rad_s:
                                primary_root_prediction.map(|summary| summary[3]),
                            primary_root_prediction_translation_error_radius_m:
                                primary_root_prediction.map(|summary| summary[4]),
                            primary_root_prediction_rotation_error_radius_rad:
                                primary_root_prediction.map(|summary| summary[5]),
                            primary_world_prediction_clearance_erosion_m: finite_metric(
                                command_output
                                    .primary_world_collision
                                    .maximum_prediction_clearance_erosion_m,
                            ),
                            contingency_root_prediction_translation_m:
                                contingency_root_prediction.map(|summary| summary[0]),
                            contingency_root_prediction_rotation_rad:
                                contingency_root_prediction.map(|summary| summary[1]),
                            contingency_world_prediction_clearance_erosion_m: finite_metric(
                                command_output
                                    .contingency_world_collision
                                    .maximum_prediction_clearance_erosion_m,
                            ),
                            command_admission_us: Some(maximum_command_admission_us),
                            command_admission_batch_us: Some(command_admission_batch_us),
                            task_pseudoinverse_calls: floating_output
                                .solve
                                .task_pseudoinverse_calls,
                            task_jacobi_sweeps: floating_output.solve.task_jacobi_sweeps,
                            clipped_steps: floating_output.solve.clipped_steps,
                            feasibility_projection_sweeps: floating_output
                                .solve
                                .feasibility_projection_sweeps,
                            feasibility_polish_iterations: floating_output
                                .solve
                                .feasibility_polish_iterations,
                            sample_count: 0,
                            clipped_levels: floating_output.solve.clipped_levels.clone(),
                            task_residuals: floating_output.task_residuals,
                        },
                    };
                    if send_json(&mut sender, &message).await.is_err() {
                        break;
                    }
                    tick_index += 1;
                    continue;
                }
                floating_state.robot.clone_from(&robot);
                floating_state.root_twist_world = Default::default();
                let mut frame_targets = Vec::with_capacity(1);
                if let Some(ActiveDrag::Point(target)) = drag.as_ref() {
                    frame_targets.push(FrameTarget {
                        stable_id: 1,
                        frame: target.frame,
                        point_in_frame: Vec3::zeros(),
                        target_position_world: target.target,
                        target_orientation_world: None,
                        feedforward_linear_velocity_world: Vec3::zeros(),
                        priority: Priority::Intent,
                        weight: 1.0,
                    });
                }
                let mut interaction_posture = standing_posture.clone();
                if let Some(ActiveDrag::Joint {
                    coordinate,
                    target_position,
                    ..
                }) = drag.as_ref()
                {
                    interaction_posture[*coordinate] = *target_position;
                }
                let start = Instant::now();
                match controller.advance_into(
                    ControllerInput {
                        tick_time_ns: tick_index as i64 * 20_000_000,
                        state: robot.clone(),
                        signal_inputs: Default::default(),
                        frame_targets,
                        com_target_world: None,
                        posture_target: Some(interaction_posture),
                        hard_constraints: vec![],
                    },
                    &controller_state,
                    &mut controller_state_next,
                    &mut output_buffer,
                    &mut scratch,
                ) {
                    Ok(bonesaw_core::StepStatus::Rejected) => {
                        let message = ServerMessage::Error {
                            message: "controller rejected program epoch".into(),
                        };
                        if send_json(&mut sender, &message).await.is_err() {
                            break;
                        }
                    }
                    Ok(_) => {
                        let Some(output) = output_buffer.value.as_ref() else {
                            let message = ServerMessage::Error {
                                message: "controller returned no output".into(),
                            };
                            if send_json(&mut sender, &message).await.is_err() {
                                break;
                            }
                            continue;
                        };
                        let solve_us = start.elapsed().as_secs_f64() * 1e6;
                        let interaction_priority = if matches!(
                            drag.as_ref(),
                            Some(ActiveDrag::Joint { .. })
                        ) {
                            Priority::Preference
                        } else {
                            Priority::Intent
                        };
                        let intent_residual = output
                            .solve
                            .level_residuals
                            .iter()
                            .find(|level| level.priority == interaction_priority)
                            .map_or(0.0, |level| level.l2);
                        let joint_authority = minimum_joint_position_headroom(
                            &app.program.model,
                            &output.next_state,
                        )
                        .ok()
                        .flatten();
                        let joint_velocity_stopping_headroom = velocity_stopping_headrooms(
                            &app.program.model,
                            &output.next_state,
                            kinematic_max_acceleration,
                        );
                        let joint_velocity_stopping_authority =
                            joint_velocity_stopping_headroom
                                .iter()
                                .enumerate()
                                .filter_map(|(coordinate, headroom)| {
                                    headroom.map(|headroom| (coordinate, headroom))
                                })
                                .min_by(|left, right| left.1.total_cmp(&right.1));
                        let minimum_collision_ground_clearance_m =
                            ground_root_lift(&app.program.model, &output.next_state)
                                .ok()
                                .map(|lift| -lift);
                        let message = ServerMessage::State {
                            tick: tick_index,
                            reset_epoch,
                            status: output.status,
                            frames: output.frames.clone(),
                            joint_positions: output.next_state.q.as_slice().to_vec(),
                            joint_velocities: output.next_state.v.as_slice().to_vec(),
                            commanded_joint_velocity: Some(
                                output.commanded_velocity.as_slice().to_vec(),
                            ),
                            command_id,
                            active_frame: drag
                                .as_ref()
                                .map(|target| target.frame_name().to_owned()),
                            metrics: Metrics {
                                authority_profile: "kinematic_controller",
                                solve_us,
                                query_batch_us: solve_us,
                                query_count: 1,
                                guided_preview_wbc_admitted: None,
                                interaction_target_clamped: false,
                                interaction_target_clamp_error_m: 0.0,
                                minimum_collision_ground_clearance_m,
                                intent_residual,
                                trajectory_velocity_scale: Some(
                                    output.trajectory_velocity_scale,
                                ),
                                trajectory_backtrack_steps: Some(
                                    output.trajectory_backtrack_steps,
                                ),
                                min_bound_margin: output.solve.minimum_bound_margin,
                                active_constraint_rows: output
                                    .solve
                                    .active_constraints
                                    .clone(),
                                dynamics_residual_linf: None,
                                contact_residual_linf: None,
                                maximum_constraint_violation: finite_metric(
                                    output.solve.maximum_constraint_violation,
                                ),
                                raw_minimum_support_margin_m: None,
                                minimum_support_margin_m: None,
                                limiting_support_patch: None,
                                collision_barrier_minimum_distance_m: None,
                                raw_collision_barrier_minimum_margin_m: None,
                                collision_barrier_minimum_margin_m: None,
                                collision_barrier_closest_pair: None,
                                collision_barrier_closest_body_a: None,
                                collision_barrier_closest_body_b: None,
                                collision_barrier_active_pairs: 0,
                                collision_barrier_limiting_pair: None,
                                collision_barrier_limiting_body_a: None,
                                collision_barrier_limiting_body_b: None,
                                collision_barrier_quality: None,
                                collision_barrier_relative_velocity_mps: None,
                                collision_barrier_required_acceleration_mps2: None,
                                collision_barrier_achieved_acceleration_mps2: None,
                                collision_barrier_residual_mps2: None,
                                collision_barrier_unsupported_shapes: 0,
                                world_collision_minimum_distance_m: None,
                                raw_world_collision_minimum_margin_m: None,
                                world_collision_minimum_margin_m: None,
                                world_collision_closest_probe: None,
                                world_collision_closest_body: None,
                                world_collision_active_probes: 0,
                                world_collision_limiting_probe: None,
                                world_collision_limiting_body: None,
                                world_collision_field_source: None,
                                world_collision_proxy_quality: None,
                                world_collision_gradient_norm: None,
                                world_collision_relative_velocity_mps: None,
                                world_collision_required_acceleration_mps2: None,
                                world_collision_achieved_acceleration_mps2: None,
                                world_collision_residual_mps2: None,
                                world_collision_unsupported_shapes: 0,
                                world_collision_outside_policy: "reject",
                                center_of_mass_world: None,
                                minimum_friction_margin: None,
                                limiting_actuator: None,
                                maximum_torque_utilization: None,
                                limiting_joint: joint_authority
                                    .map(|authority| authority.coordinate),
                                raw_minimum_joint_margin_rad: joint_authority
                                    .map(|authority| authority.margin_rad),
                                minimum_joint_margin_rad: joint_authority
                                    .map(|authority| authority.margin_rad),
                                minimum_joint_headroom_fraction: joint_authority
                                    .map(|authority| authority.fraction_of_range),
                                joint_position_headroom_rad: joint_position_headrooms(
                                    &app.program.model,
                                    &output.next_state,
                                ),
                                limiting_joint_stopping: None,
                                raw_minimum_joint_stopping_margin_rad_s2: None,
                                minimum_joint_stopping_margin_rad_s2: None,
                                joint_stopping_recovery_count: 0,
                                limiting_joint_velocity_stopping:
                                    joint_velocity_stopping_authority
                                        .map(|authority| authority.0),
                                minimum_joint_velocity_stopping_headroom_rad:
                                    joint_velocity_stopping_authority
                                        .map(|authority| authority.1),
                                joint_velocity_stopping_headroom_rad:
                                    joint_velocity_stopping_headroom,
                                command_selection: None,
                                command_admission_flags: None,
                                command_tracking_action: None,
                                command_tracking_position_error: None,
                                command_tracking_velocity_error: None,
                                command_tracking_limiting_position_actuator: None,
                                command_tracking_limiting_velocity_actuator: None,
                                command_tracking_position_contingency_headroom: None,
                                command_tracking_position_reject_headroom: None,
                                command_tracking_velocity_contingency_headroom: None,
                                command_tracking_velocity_reject_headroom: None,
                                robot_observation_source_time_ns: None,
                                robot_observation_mapped_time_ns: None,
                                robot_observation_source_sequence: None,
                                robot_observation_source_id: None,
                                robot_observation_synchronization_uncertainty_ns: None,
                                robot_observation_age_ns: None,
                                robot_observation_age_headroom_ns: None,
                                robot_observation_synchronization_headroom_ns: None,
                                robot_observation_causal: None,
                                robot_observation_age_valid: None,
                                robot_observation_synchronization_valid: None,
                                robot_observation_history_len: robot_observation_history.len(),
                                robot_observation_history_capacity: robot_observation_history
                                    .capacity(),
                                robot_observation_ingest_accepted: 0,
                                robot_observation_ingest_ignored: 0,
                                robot_observation_ingest_rejected: 0,
                                robot_observation_transport_mode:
                                    observation_transport_mode.label(),
                                robot_observation_transport_query_time_ns: None,
                                robot_observation_transport_sample_emitted: false,
                                robot_observation_frame_exact_queries: 0,
                                robot_observation_frame_interpolated_queries: 0,
                                robot_observation_frame_predicted_queries: 0,
                                robot_observation_frame_held_queries: 0,
                                robot_observation_error_exposure_ns: None,
                                robot_observation_joint_position_error_rad: None,
                                robot_observation_joint_velocity_error_rad_s: None,
                                robot_observation_root_translation_error_m: None,
                                robot_observation_root_rotation_error_rad: None,
                                robot_observation_point_position_error_m: None,
                                robot_observation_center_of_mass_position_error_m: None,
                                robot_observation_reconstruction_provenance: None,
                                robot_observation_reconstruction_lower_time_ns: None,
                                robot_observation_reconstruction_upper_time_ns: None,
                                robot_observation_reconstruction_lower_source_id: None,
                                robot_observation_reconstruction_upper_source_id: None,
                                robot_observation_reconstruction_lower_sequence: None,
                                robot_observation_reconstruction_upper_sequence: None,
                                robot_observation_reconstruction_source_age_ns: None,
                                robot_observation_reconstruction_source_age_headroom_ns: None,
                                robot_observation_reconstruction_synchronization_headroom_ns: None,
                                robot_observation_reconstruction_hard_eligible: None,
                                command_clearance_requirement_m: Some(
                                    LIVE_SELF_COLLISION_CLEARANCE_M,
                                ),
                                primary_sampled_clearance_m: None,
                                primary_robust_sampled_clearance_m: None,
                                primary_continuous_clearance_m: None,
                                primary_robust_continuous_clearance_m: None,
                                primary_continuous_limiting_pair: None,
                                primary_continuous_limiting_body_a: None,
                                primary_continuous_limiting_body_b: None,
                                primary_continuous_relative_speed_m_s: None,
                                primary_continuity_leaf_intervals: 0,
                                primary_refinement_pair_samples: 0,
                                primary_continuity_unresolved_intervals: 0,
                                primary_continuity_maximum_subdivision_depth: 0,
                                primary_minimum_collision_pair: None,
                                primary_minimum_collision_body_a: None,
                                primary_minimum_collision_body_b: None,
                                primary_first_collision_pair: None,
                                primary_first_collision_body_a: None,
                                primary_first_collision_body_b: None,
                                primary_first_collision_time_ns: None,
                                contingency_sampled_clearance_m: None,
                                contingency_robust_sampled_clearance_m: None,
                                contingency_continuous_clearance_m: None,
                                contingency_robust_continuous_clearance_m: None,
                                contingency_continuous_limiting_pair: None,
                                contingency_continuous_limiting_body_a: None,
                                contingency_continuous_limiting_body_b: None,
                                contingency_continuous_relative_speed_m_s: None,
                                contingency_continuity_leaf_intervals: 0,
                                contingency_refinement_pair_samples: 0,
                                contingency_continuity_unresolved_intervals: 0,
                                contingency_continuity_maximum_subdivision_depth: 0,
                                primary_world_sampled_clearance_m: None,
                                primary_world_robust_sampled_clearance_m: None,
                                primary_world_minimum_probe: None,
                                primary_world_minimum_body: None,
                                primary_world_field_source: None,
                                primary_world_first_violation_probe: None,
                                primary_world_first_violation_body: None,
                                primary_world_first_violation_time_ns: None,
                                primary_world_first_unknown_probe: None,
                                primary_world_first_unknown_body: None,
                                primary_world_first_unknown_time_ns: None,
                                primary_world_continuous_clearance_m: None,
                                primary_world_robust_continuous_clearance_m: None,
                                primary_world_continuous_limiting_probe: None,
                                primary_world_continuous_limiting_body: None,
                                primary_world_distance_rate_bound_m_s: None,
                                primary_world_leaf_intervals: 0,
                                primary_world_refinement_probe_samples: 0,
                                primary_world_unresolved_intervals: 0,
                                primary_world_maximum_subdivision_depth: 0,
                                contingency_world_sampled_clearance_m: None,
                                contingency_world_robust_sampled_clearance_m: None,
                                contingency_world_minimum_probe: None,
                                contingency_world_minimum_body: None,
                                contingency_world_field_source: None,
                                contingency_world_first_violation_probe: None,
                                contingency_world_first_violation_body: None,
                                contingency_world_first_violation_time_ns: None,
                                contingency_world_first_unknown_probe: None,
                                contingency_world_first_unknown_body: None,
                                contingency_world_first_unknown_time_ns: None,
                                contingency_world_continuous_clearance_m: None,
                                contingency_world_robust_continuous_clearance_m: None,
                                contingency_world_continuous_limiting_probe: None,
                                contingency_world_continuous_limiting_body: None,
                                contingency_world_distance_rate_bound_m_s: None,
                                contingency_world_leaf_intervals: 0,
                                contingency_world_refinement_probe_samples: 0,
                                contingency_world_unresolved_intervals: 0,
                                contingency_world_maximum_subdivision_depth: 0,
                                world_scene_epoch: None,
                                world_scene_source_time_ns: None,
                                world_scene_age_ns: None,
                                world_scene_valid_until_ns: None,
                                world_scene_horizon_end_ns: None,
                                world_scene_validity: None,
                                primary_root_prediction_translation_m: None,
                                primary_root_prediction_rotation_rad: None,
                                primary_root_prediction_max_linear_speed_m_s: None,
                                primary_root_prediction_max_angular_speed_rad_s: None,
                                primary_root_prediction_translation_error_radius_m: None,
                                primary_root_prediction_rotation_error_radius_rad: None,
                                primary_world_prediction_clearance_erosion_m: None,
                                contingency_root_prediction_translation_m: None,
                                contingency_root_prediction_rotation_rad: None,
                                contingency_world_prediction_clearance_erosion_m: None,
                                command_admission_us: None,
                                command_admission_batch_us: None,
                                task_pseudoinverse_calls: output
                                    .solve
                                    .task_pseudoinverse_calls,
                                task_jacobi_sweeps: output.solve.task_jacobi_sweeps,
                                clipped_steps: output.solve.clipped_steps,
                                feasibility_projection_sweeps: output
                                    .solve
                                    .feasibility_projection_sweeps,
                                feasibility_polish_iterations: output
                                    .solve
                                    .feasibility_polish_iterations,
                                sample_count: output.samples.len(),
                                clipped_levels: output.solve.clipped_levels.clone(),
                                task_residuals: [FloatingTaskResidual::inactive();
                                    FLOATING_TASK_DIAGNOSTIC_CAPACITY],
                            },
                        };
                        robot.clone_from(&output.next_state);
                        std::mem::swap(
                            &mut controller_state,
                            &mut controller_state_next,
                        );
                        if send_json(&mut sender, &message).await.is_err() {
                            break;
                        }
                    }
                    Err(error) => {
                        let message = ServerMessage::Error {
                            message: error.to_string(),
                        };
                        if send_json(&mut sender, &message).await.is_err() {
                            break;
                        }
                    }
                }
                tick_index += 1;
            }
        }
    }
}

fn collect_render_geometry(model: &CompiledModel) -> Vec<RenderGeometry> {
    model
        .bodies
        .iter()
        .flat_map(|body| {
            body.collisions
                .iter()
                .map(|shape| render_geometry(body.id.0, shape))
        })
        .collect()
}

fn collect_render_visuals(model: &CompiledModel) -> Vec<RenderVisual> {
    model
        .bodies
        .iter()
        .flat_map(|body| {
            body.visuals.iter().map(|visual| RenderVisual {
                shape: render_geometry(body.id.0, &visual.geometry),
                rgba: visual.rgba,
            })
        })
        .collect()
}

fn render_geometry(body: usize, shape: &CollisionShape) -> RenderGeometry {
    let body_from_shape = match shape {
        CollisionShape::Sphere {
            body_from_shape, ..
        }
        | CollisionShape::Capsule {
            body_from_shape, ..
        }
        | CollisionShape::Box {
            body_from_shape, ..
        }
        | CollisionShape::Cylinder {
            body_from_shape, ..
        }
        | CollisionShape::Mesh {
            body_from_shape, ..
        } => body_from_shape,
    };
    let translation = body_from_shape.translation.vector;
    let translation = [translation.x, translation.y, translation.z];
    let quaternion = body_from_shape.rotation.quaternion();
    let rotation_xyzw = [quaternion.i, quaternion.j, quaternion.k, quaternion.w];
    match shape {
        CollisionShape::Sphere { radius, .. } => RenderGeometry::Sphere {
            body,
            radius: *radius,
            translation,
            rotation_xyzw,
        },
        CollisionShape::Capsule {
            radius,
            half_length,
            ..
        } => RenderGeometry::Capsule {
            body,
            radius: *radius,
            half_length: *half_length,
            translation,
            rotation_xyzw,
        },
        CollisionShape::Box { half_extents, .. } => RenderGeometry::Box {
            body,
            half_extents: [half_extents.x, half_extents.y, half_extents.z],
            translation,
            rotation_xyzw,
        },
        CollisionShape::Cylinder {
            radius,
            half_length,
            ..
        } => RenderGeometry::Cylinder {
            body,
            radius: *radius,
            half_length: *half_length,
            translation,
            rotation_xyzw,
        },
        CollisionShape::Mesh {
            filename, scale, ..
        } => RenderGeometry::Mesh {
            body,
            filename: filename.clone(),
            scale: [scale.x, scale.y, scale.z],
            translation,
            rotation_xyzw,
        },
    }
}

fn ground_root_lift(
    model: &CompiledModel,
    state: &RobotState,
) -> Result<f64, bonesaw_core::model::ModelError> {
    let mut cache = ModelCache::new(model);
    model.forward_kinematics(state, &mut cache)?;
    let mut minimum_z = f64::INFINITY;
    for body in &model.bodies {
        for shape in &body.collisions {
            let (body_from_shape, vertical_extent) = match shape {
                CollisionShape::Sphere {
                    radius,
                    body_from_shape,
                } => (*body_from_shape, *radius),
                CollisionShape::Capsule {
                    radius,
                    half_length,
                    body_from_shape,
                } => {
                    let world_from_shape = cache.world_from_body[body.id.0] * body_from_shape;
                    let axis_z = world_from_shape.rotation * Vec3::z();
                    (*body_from_shape, radius + half_length * axis_z.z.abs())
                }
                CollisionShape::Box {
                    half_extents,
                    body_from_shape,
                } => {
                    let world_from_shape = cache.world_from_body[body.id.0] * body_from_shape;
                    let rotation = world_from_shape.rotation.to_rotation_matrix();
                    let extent = rotation[(2, 0)].abs() * half_extents.x
                        + rotation[(2, 1)].abs() * half_extents.y
                        + rotation[(2, 2)].abs() * half_extents.z;
                    (*body_from_shape, extent)
                }
                CollisionShape::Cylinder {
                    radius,
                    half_length,
                    body_from_shape,
                } => {
                    let world_from_shape = cache.world_from_body[body.id.0] * body_from_shape;
                    let axis_z = world_from_shape.rotation * Vec3::z();
                    let radial_z = (1.0 - axis_z.z * axis_z.z).max(0.0).sqrt();
                    (
                        *body_from_shape,
                        half_length * axis_z.z.abs() + radius * radial_z,
                    )
                }
                CollisionShape::Mesh { .. } => continue,
            };
            let world_from_shape = cache.world_from_body[body.id.0] * body_from_shape;
            minimum_z = minimum_z.min(world_from_shape.translation.vector.z - vertical_extent);
        }
    }
    Ok(if minimum_z.is_finite() {
        -minimum_z
    } else {
        0.0
    })
}

fn fingerprint_hex(fingerprint: &[u8; 32]) -> String {
    fingerprint
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

async fn send_json(
    sender: &mut futures_util::stream::SplitSink<WebSocket, Message>,
    message: &ServerMessage,
) -> Result<(), axum::Error> {
    sender
        .send(Message::Text(
            serde_json::to_string(message)
                .expect("server message serialization is infallible")
                .into(),
        ))
        .await
}

fn standing_posture(model: &CompiledModel) -> DVector<f64> {
    let mut q = DVector::zeros(model.dof);
    let upkie = [
        ("left_hip", 0.4),
        ("right_hip", -0.4),
        ("left_knee", -0.625),
        ("right_knee", 0.625),
    ];
    let generic = [
        ("left_hip_pitch", -0.15),
        ("right_hip_pitch", -0.15),
        ("left_knee", 0.3),
        ("right_knee", 0.3),
        ("left_ankle", -0.15),
        ("right_ankle", -0.15),
        ("left_shoulder_roll", 0.25),
        ("right_shoulder_roll", -0.25),
        ("left_elbow", -0.25),
        ("right_elbow", 0.25),
    ];
    let entries: &[(&str, f64)] = if model.joint_id("left_hip").is_some() {
        &upkie
    } else {
        &generic
    };
    for &(name, value) in entries {
        if let Some(joint) = model.joint_id(name)
            && let Some(index) = model.joints[joint.0].coordinate
        {
            q[index] = value;
        }
    }
    q
}

fn actuator_torque_bounds(program: &MotionProgram) -> Result<VelocityBounds> {
    let model = &program.model;
    let effort_limits = program
        .actuation
        .independent_generalized_effort_limits()
        .ok_or_else(|| anyhow!("floating editor requires an independent diagonal actuation map"))?;
    let mut bounds = VelocityBounds {
        lower: DVector::from_element(model.dof, -1_000.0),
        upper: DVector::from_element(model.dof, 1_000.0),
    };
    for (coordinate, effort) in effort_limits.into_iter().enumerate() {
        let effort = effort.min(1_000.0);
        bounds.lower[coordinate] = -effort;
        bounds.upper[coordinate] = effort;
    }
    Ok(bounds)
}

fn finite_metric(value: f64) -> Option<f64> {
    value.is_finite().then_some(value)
}

fn world_scene_validity_label(validity: WorldSceneValidity) -> &'static str {
    match validity {
        WorldSceneValidity::Valid => "valid",
        WorldSceneValidity::EpochMismatch => "epoch_mismatch",
        WorldSceneValidity::SourceFromFuture => "source_from_future",
        WorldSceneValidity::NotYetValid => "not_yet_valid",
        WorldSceneValidity::ExpiredAtTick => "expired_at_tick",
        WorldSceneValidity::HorizonExpired => "horizon_expired",
        WorldSceneValidity::TooOld => "too_old",
        WorldSceneValidity::InvalidStamp => "invalid_stamp",
    }
}

fn root_prediction_summary(segment: &RootPosePredictionSegment) -> Option<[f64; 6]> {
    let end_time_ns = segment.start_time_ns.saturating_add(segment.duration_ns);
    let mut end_pose = bonesaw_core::Transform3::identity();
    let mut end_twist = Motion6::default();
    let mut end_acceleration = SpatialAcceleration6::default();
    segment
        .evaluate_into(
            end_time_ns,
            &mut end_pose,
            &mut end_twist,
            &mut end_acceleration,
        )
        .ok()?;
    let mut maximum_abs_twist = Motion6::default();
    segment
        .maximum_abs_twist_between_into(segment.start_time_ns, end_time_ns, &mut maximum_abs_twist)
        .ok()?;
    let translation = (end_pose.translation.vector
        - segment.control_world_from_root_at_start.translation.vector)
        .norm();
    let rotation =
        (end_pose.rotation * segment.control_world_from_root_at_start.rotation.inverse()).angle();
    let angular_speed = Vec3::new(
        maximum_abs_twist.0[0],
        maximum_abs_twist.0[1],
        maximum_abs_twist.0[2],
    )
    .norm();
    let linear_speed = Vec3::new(
        maximum_abs_twist.0[3],
        maximum_abs_twist.0[4],
        maximum_abs_twist.0[5],
    )
    .norm();
    let (translation_error_radius, rotation_error_radius) =
        segment.error_radii_at(end_time_ns).ok()?;
    [
        translation,
        rotation,
        linear_speed,
        angular_speed,
        translation_error_radius,
        rotation_error_radius,
    ]
    .iter()
    .all(|value| value.is_finite())
    .then_some([
        translation,
        rotation,
        linear_speed,
        angular_speed,
        translation_error_radius,
        rotation_error_radius,
    ])
}

fn editor_world_collision(
    model: &CompiledModel,
) -> Result<CompiledWorldCollisionModel, bonesaw_core::WorldCollisionError> {
    // A static vertical plane beside the toy rig makes world authority visible
    // without introducing a physics engine. The finite grid is deliberately
    // larger than every guided pose; leaving it is a typed query error.
    let dimensions = [25, 21, 31];
    let spacing = Vec3::repeat(0.1);
    let origin = Vec3::new(-1.2, -1.0, -0.5);
    let mut samples = Vec::with_capacity(dimensions[0] * dimensions[1] * dimensions[2]);
    for _z in 0..dimensions[2] {
        for y in 0..dimensions[1] {
            let world_y = origin.y + y as f64 * spacing.y;
            for _x in 0..dimensions[0] {
                samples.push(LIVE_WORLD_WALL_Y_M - world_y);
            }
        }
    }
    let field = DenseSdfGrid::new(
        bonesaw_core::Transform3::from_parts(
            Translation3::from(origin),
            UnitQuaternion::identity(),
        ),
        dimensions,
        spacing,
        samples,
        SdfOutsidePolicy::Reject,
    )?;
    CompiledWorldCollisionModel::compile(model, field)
}

fn distance_quality_label(quality: DistanceQuality) -> &'static str {
    match quality {
        DistanceQuality::ExactSphere => "exact_sphere",
        DistanceQuality::AnalyticPrimitive => "analytic_primitive",
        DistanceQuality::ConservativePrimitive => "conservative_primitive",
        DistanceQuality::ConservativeBoundingSphere => "conservative_bounding_sphere",
        DistanceQuality::ConservativeSphereCover => "conservative_sphere_cover",
    }
}

fn sdf_sample_source_label(source: SdfSampleSource) -> &'static str {
    match source {
        SdfSampleSource::TrilinearGrid => "trilinear_grid",
        SdfSampleSource::OccupiedBoundary => "occupied_boundary",
    }
}

fn dynamic_selection_label(selection: DynamicPlanSelection) -> &'static str {
    match selection {
        DynamicPlanSelection::Primary => "primary",
        DynamicPlanSelection::Contingency => "contingency",
        DynamicPlanSelection::Rejected => "rejected",
    }
}

fn command_tracking_action_label(action: CommandTrackingAction) -> &'static str {
    match action {
        CommandTrackingAction::Nominal => "nominal",
        CommandTrackingAction::Contingency => "contingency",
        CommandTrackingAction::Rejected => "rejected",
    }
}

fn reconstruction_provenance_label(provenance: ReconstructionProvenance) -> &'static str {
    match provenance {
        ReconstructionProvenance::ExactSample => "exact",
        ReconstructionProvenance::Interpolated => "interpolated",
        ReconstructionProvenance::PredictedConstantVelocity => "predicted",
        ReconstructionProvenance::Held => "held",
    }
}

fn observation_ingest_ignored(report: RobotObservationIngestReport) -> usize {
    report.ignored_duplicate + report.ignored_source
}

#[allow(clippy::too_many_arguments)]
fn ingest_live_observation(
    history: &mut RobotObservationHistory,
    model: &CompiledModel,
    program_epoch: u64,
    sample_time_ns: i64,
    source_sequence: u64,
    state: &FloatingRobotState,
    mode: ObservationTransportMode,
    report: &mut RobotObservationIngestReport,
) -> Result<bool, String> {
    *report = RobotObservationIngestReport::default();
    if !mode.emits_sample(sample_time_ns, history.is_empty()) {
        return Ok(false);
    }
    let observation = RobotObservationRef {
        program_epoch,
        stamp: RobotObservationStamp::exact_at(sample_time_ns, 0xB015, source_sequence),
        state,
    };
    if mode == ObservationTransportMode::Interpolated && history.is_empty() {
        // A mode can be selected before dynamic execution has produced any
        // history. Seed one 20 ms-earlier zero-order sample so the first query
        // is a real bracketed interpolation instead of a permanent
        // BeforeHistory/20 ms-gap failure loop. The seed is test-harness
        // evidence only; later samples carry the actual producer state.
        let seed_observation = RobotObservationRef {
            program_epoch,
            stamp: RobotObservationStamp::exact_at(
                sample_time_ns.saturating_sub(20_000_000),
                0xB015,
                source_sequence.saturating_sub(1),
            ),
            state,
        };
        let observations = [seed_observation, observation];
        *report = history.ingest_batch(
            model,
            sample_time_ns,
            program_epoch,
            LIVE_ROBOT_OBSERVATION_LIMITS,
            &observations,
        );
    } else {
        *report = history.ingest_batch(
            model,
            sample_time_ns,
            program_epoch,
            LIVE_ROBOT_OBSERVATION_LIMITS,
            std::slice::from_ref(&observation),
        );
    }
    if report.rejected() != 0 || (report.accepted() == 0 && report.ignored_duplicate != 1) {
        return Err(format!(
            "robot observation ingest withheld WBC input: {report:?}"
        ));
    }
    Ok(true)
}

fn record_reconstruction_query(counts: &mut [usize; 4], provenance: ReconstructionProvenance) {
    let index = match provenance {
        ReconstructionProvenance::ExactSample => 0,
        ReconstructionProvenance::Interpolated => 1,
        ReconstructionProvenance::PredictedConstantVelocity => 2,
        ReconstructionProvenance::Held => 3,
    };
    counts[index] += 1;
}

fn compose_joint_stopping_bounds(
    model: &CompiledModel,
    state: &RobotState,
    position_error_rad: f64,
    velocity_error_rad_s: f64,
    maximum_acceleration: f64,
    dt_seconds: f64,
    bounds: &mut VelocityBounds,
) -> Result<usize> {
    if bounds.lower.len() != model.dof + 6 || bounds.upper.len() != model.dof + 6 {
        anyhow::bail!("joint-stopping bounds have the wrong generalized dimension");
    }
    bounds.lower.fill(-maximum_acceleration);
    bounds.upper.fill(maximum_acceleration);
    let mut recovery_count = 0;
    for joint in &model.joints {
        let Some(coordinate) = joint.coordinate else {
            continue;
        };
        let Some((lower, upper)) = joint_acceleration_interval_with_observation_error(
            state.q[coordinate],
            state.v[coordinate],
            position_error_rad,
            velocity_error_rad_s,
            joint.limit.lower,
            joint.limit.upper,
            joint.limit.velocity,
            maximum_acceleration,
            dt_seconds,
        ) else {
            anyhow::bail!("joint {} has no valid stopping envelope", joint.name);
        };
        bounds.lower[6 + coordinate] = bounds.lower[6 + coordinate].max(lower);
        bounds.upper[6 + coordinate] = bounds.upper[6 + coordinate].min(upper);
        if state.q[coordinate] < joint.limit.lower || state.q[coordinate] > joint.limit.upper {
            recovery_count += 1;
        }
    }
    Ok(recovery_count)
}

fn minimum_joint_stopping_margin(
    generalized_acceleration: &DVector<f64>,
    bounds: &VelocityBounds,
    dof: usize,
) -> Option<(usize, f64)> {
    (0..dof)
        .map(|coordinate| {
            let generalized = 6 + coordinate;
            let acceleration = generalized_acceleration[generalized];
            (
                coordinate,
                (acceleration - bounds.lower[generalized])
                    .min(bounds.upper[generalized] - acceleration),
            )
        })
        .filter(|(_, margin)| margin.is_finite())
        .min_by(|left, right| left.1.total_cmp(&right.1))
}

#[allow(clippy::too_many_arguments)]
fn prepare_balanced_upkie_squat_target(
    model: &CompiledModel,
    current: &RobotState,
    standing: &DVector<f64>,
    target_root: Vec3,
    targets: &[PlanarPointIkTarget; 2],
    _support_targets: &[(FrameId, Vec3)],
    target: &mut RobotState,
    scratch: &mut PlanarIkScratch,
    cache: &mut ModelCache,
) -> Result<()> {
    target.clone_from(current);
    target.control_world_from_root.rotation = Default::default();
    target.control_world_from_root.translation.vector = target_root;
    target.q.copy_from(standing);
    let report =
        solve_planar_point_ik_into(model, target, targets, PlanarIkOptions::default(), scratch)?;
    if !report.converged {
        anyhow::bail!(
            "contact IK failed to converge: {:.3} mm",
            report.maximum_planar_error_m * 1000.0
        );
    }
    model.forward_kinematics(target, cache)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn prepare_ground_safe_balanced_upkie_squat_target(
    model: &CompiledModel,
    current: &RobotState,
    standing: &DVector<f64>,
    standing_root: Vec3,
    requested_root: Vec3,
    targets: &[PlanarPointIkTarget; 2],
    support_targets: &[(FrameId, Vec3)],
    target: &mut RobotState,
    scratch: &mut PlanarIkScratch,
    cache: &mut ModelCache,
) -> Result<(Vec3, f64)> {
    const GROUND_TOLERANCE_M: f64 = 1.0e-6;
    const SEARCH_STEPS: usize = 16;

    let requested_is_safe = prepare_balanced_upkie_squat_target(
        model,
        current,
        standing,
        requested_root,
        targets,
        support_targets,
        target,
        scratch,
        cache,
    )
    .is_ok_and(|()| {
        ground_root_lift(model, target)
            .is_ok_and(|lift| lift <= GROUND_TOLERANCE_M - LIVE_GROUND_REGISTRATION_MARGIN_M)
    });
    if requested_is_safe {
        return Ok((requested_root, 0.0));
    }

    prepare_balanced_upkie_squat_target(
        model,
        current,
        standing,
        standing_root,
        targets,
        support_targets,
        target,
        scratch,
        cache,
    )?;
    let standing_lift = ground_root_lift(model, target)?;
    if standing_lift > GROUND_TOLERANCE_M - LIVE_GROUND_REGISTRATION_MARGIN_M {
        anyhow::bail!(
            "standing collision envelope misses the {:.3} mm ground-registration margin by {:.3} mm",
            LIVE_GROUND_REGISTRATION_MARGIN_M * 1000.0,
            (standing_lift + LIVE_GROUND_REGISTRATION_MARGIN_M) * 1000.0
        );
    }

    let mut safe_fraction = 0.0;
    let mut unsafe_fraction = 1.0;
    for _ in 0..SEARCH_STEPS {
        let fraction = 0.5 * (safe_fraction + unsafe_fraction);
        let candidate = standing_root + (requested_root - standing_root) * fraction;
        let candidate_is_safe = prepare_balanced_upkie_squat_target(
            model,
            current,
            standing,
            candidate,
            targets,
            support_targets,
            target,
            scratch,
            cache,
        )
        .is_ok_and(|()| {
            ground_root_lift(model, target)
                .is_ok_and(|lift| lift <= GROUND_TOLERANCE_M - LIVE_GROUND_REGISTRATION_MARGIN_M)
        });
        if candidate_is_safe {
            safe_fraction = fraction;
        } else {
            unsafe_fraction = fraction;
        }
    }
    let applied_root = standing_root + (requested_root - standing_root) * safe_fraction;
    prepare_balanced_upkie_squat_target(
        model,
        current,
        standing,
        applied_root,
        targets,
        support_targets,
        target,
        scratch,
        cache,
    )?;
    Ok((applied_root, (requested_root - applied_root).norm()))
}

fn prepare_joint_posture_acceleration(
    state: &FloatingRobotState,
    target: &RobotState,
    desired: &mut DVector<f64>,
) {
    for index in 0..state.robot.q.len() {
        desired[6 + index] = (30.0 * (target.q[index] - state.robot.q[index])
            - 10.0 * state.robot.v[index])
            .clamp(-50.0, 50.0);
    }
}

fn compile_upkie_planar_ik_targets(
    model: &CompiledModel,
    squat: &SquatContext,
) -> Option<[PlanarPointIkTarget; 2]> {
    let left_hip = model
        .joint_id("left_hip")
        .and_then(|joint| model.joints[joint.0].coordinate)?;
    let left_knee = model
        .joint_id("left_knee")
        .and_then(|joint| model.joints[joint.0].coordinate)?;
    let right_hip = model
        .joint_id("right_hip")
        .and_then(|joint| model.joints[joint.0].coordinate)?;
    let right_knee = model
        .joint_id("right_knee")
        .and_then(|joint| model.joints[joint.0].coordinate)?;
    Some([
        PlanarPointIkTarget {
            frame: squat.support_targets[0].0,
            point_in_frame: Vec3::zeros(),
            target_world: squat.support_targets[0].1,
            coordinates: [left_hip, left_knee],
            axes: [0, 2],
        },
        PlanarPointIkTarget {
            frame: squat.support_targets[1].0,
            point_in_frame: Vec3::zeros(),
            target_world: squat.support_targets[1].1,
            coordinates: [right_hip, right_knee],
            axes: [0, 2],
        },
    ])
}

fn authored_squat_posture(
    model: &CompiledModel,
    squat: &SquatContext,
    target_root_z: f64,
    standing: &DVector<f64>,
    target: &mut DVector<f64>,
) {
    target.copy_from(standing);
    let squat_fraction = ((squat.standing_root.z - target_root_z) / 0.12).clamp(0.0, 1.0);
    for (name, offset) in [
        ("left_hip_pitch", -0.42),
        ("right_hip_pitch", -0.42),
        ("left_knee", 0.84),
        ("right_knee", 0.84),
        ("left_ankle", -0.42),
        ("right_ankle", -0.42),
    ] {
        let Some(index) = model
            .joint_id(name)
            .and_then(|joint| model.joints[joint.0].coordinate)
        else {
            continue;
        };
        target[index] = standing[index] + squat_fraction * offset;
    }
}

fn stabilize_contacts(
    model: &CompiledModel,
    state: &FloatingRobotState,
    contacts: &mut [ContactSpec],
    targets: &[Vec3],
    cache: &mut ModelCache,
    jacobian: &mut DMatrix<f64>,
    generalized_velocity: &mut DVector<f64>,
) -> Result<(), bonesaw_core::model::ModelError> {
    debug_assert_eq!(contacts.len(), targets.len());
    model.forward_kinematics(&state.robot, cache)?;
    for axis in 0..6 {
        generalized_velocity[axis] = state.root_twist_world.0[axis];
    }
    generalized_velocity
        .rows_mut(6, model.dof)
        .copy_from(&state.robot.v);
    for (contact, target) in contacts.iter_mut().zip(targets) {
        model.floating_point_jacobian_into(
            cache,
            contact.frame,
            contact.point_in_frame,
            jacobian,
        )?;
        let point = cache.world_from_body[contact.frame.0]
            .transform_point(&Point3::from(contact.point_in_frame))
            .coords;
        let velocity = Vec3::new(
            (0..generalized_velocity.len())
                .map(|column| jacobian[(0, column)] * generalized_velocity[column])
                .sum(),
            (0..generalized_velocity.len())
                .map(|column| jacobian[(1, column)] * generalized_velocity[column])
                .sum(),
            (0..generalized_velocity.len())
                .map(|column| jacobian[(2, column)] * generalized_velocity[column])
                .sum(),
        );
        contact.desired_point_acceleration_world =
            (40.0 * (*target - point) - 14.0 * velocity).map(|value| value.clamp(-8.0, 8.0));
        if matches!(contact.mode, ContactMode::RollingWheel { .. }) {
            let rolling_axis = contact.tangent_x_world;
            contact.desired_point_acceleration_world -=
                rolling_axis * rolling_axis.dot(&contact.desired_point_acceleration_world);
        }
    }
    Ok(())
}

fn collect_frames(
    model: &CompiledModel,
    state: &RobotState,
    cache: &mut ModelCache,
    frames: &mut Vec<bonesaw_core::controller::FrameState>,
) -> Result<(), bonesaw_core::model::ModelError> {
    model.forward_kinematics(state, cache)?;
    frames.clear();
    for body in &model.bodies {
        let pose = cache.world_from_body[body.id.0];
        let quaternion = pose.rotation.quaternion();
        frames.push(bonesaw_core::controller::FrameState {
            id: body.id.0,
            translation: [pose.translation.x, pose.translation.y, pose.translation.z],
            rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
        });
    }
    Ok(())
}

fn joint_position_headrooms(model: &CompiledModel, state: &RobotState) -> Vec<Option<f64>> {
    let mut headrooms = vec![None; model.dof];
    for joint in &model.joints {
        let Some(coordinate) = joint.coordinate else {
            continue;
        };
        let lower = joint
            .limit
            .lower
            .is_finite()
            .then(|| state.q[coordinate] - joint.limit.lower);
        let upper = joint
            .limit
            .upper
            .is_finite()
            .then(|| joint.limit.upper - state.q[coordinate]);
        headrooms[coordinate] = match (lower, upper) {
            (Some(lower), Some(upper)) => Some(lower.min(upper)),
            (Some(lower), None) => Some(lower),
            (None, Some(upper)) => Some(upper),
            (None, None) => None,
        };
    }
    headrooms
}

fn velocity_stopping_headrooms(
    model: &CompiledModel,
    state: &RobotState,
    maximum_acceleration: f64,
) -> Vec<Option<f64>> {
    let mut headrooms = vec![None; model.dof];
    if !maximum_acceleration.is_finite() || maximum_acceleration <= 0.0 {
        return headrooms;
    }
    for joint in &model.joints {
        let Some(coordinate) = joint.coordinate else {
            continue;
        };
        let position = state.q[coordinate];
        let velocity = state.v[coordinate];
        let lower_headroom = joint.limit.lower.is_finite().then(|| {
            position - joint.limit.lower - velocity.min(0.0).powi(2) / (2.0 * maximum_acceleration)
        });
        let upper_headroom = joint.limit.upper.is_finite().then(|| {
            joint.limit.upper - position - velocity.max(0.0).powi(2) / (2.0 * maximum_acceleration)
        });
        headrooms[coordinate] = match (lower_headroom, upper_headroom) {
            (Some(lower), Some(upper)) => Some(lower.min(upper)),
            (Some(lower), None) => Some(lower),
            (None, Some(upper)) => Some(upper),
            (None, None) => None,
        };
    }
    headrooms
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interaction_commands_preserve_legacy_release_and_correlated_joint_drag() {
        let release = serde_json::from_str::<ClientCommand>(r#"{"type":"release"}"#)
            .expect("legacy release remains accepted");
        assert!(matches!(
            release,
            ClientCommand::Release { request_id: None }
        ));

        let joint_drag = serde_json::from_str::<ClientCommand>(
            r#"{"type":"joint_drag","frame":"left_ankle_mj5208_rotor","coordinate":1,"target_position":3.01,"request_id":42}"#,
        )
        .expect("correlated joint drag parses");
        assert!(matches!(
            joint_drag,
            ClientCommand::JointDrag {
                coordinate: 1,
                request_id: Some(42),
                ..
            }
        ));
    }

    #[test]
    fn plant_push_protocol_is_correlated_finite_and_force_bounded() {
        let provenance = PlantExternalLoadProvenance {
            source: PlantExternalLoadSource::EvaluationHarness,
            load_class: PlantExternalLoadClass::DeclaredContinuousWrench,
            force_frame: PlantExternalLoadFrame::World,
            application_point_frame: PlantExternalLoadFrame::World,
        };
        let push = serde_json::from_str::<PlantClientCommand>(
            r#"{"type":"plant_push","body":"base","force_world":[4.0,0.0,0.0],"application_point_world":[0.0,0.0,0.55],"provenance":{"source":"evaluation_harness","load_class":"declared_continuous_wrench","force_frame":"world","application_point_frame":"world"},"request_id":42}"#,
        )
        .expect("typed plant push parses");
        assert!(validate_plant_push(&push).is_ok());
        let mut body_positions = HashMap::new();
        body_positions.insert("base".to_owned(), [0.0, 0.0, 0.55]);
        assert!(validate_plant_application_point(&push, &body_positions).is_ok());
        assert!(matches!(
            push,
            PlantClientCommand::PlantPush { request_id: 42, .. }
        ));

        let excessive = PlantClientCommand::PlantPush {
            body: "base".to_owned(),
            force_world: [8.01, 0.0, 0.0],
            application_point_world: [0.0, 0.0, 0.55],
            provenance,
            request_id: 43,
        };
        assert!(validate_plant_push(&excessive).is_err());

        let nonfinite = PlantClientCommand::PlantPush {
            body: "base".to_owned(),
            force_world: [f64::NAN, 0.0, 0.0],
            application_point_world: [0.0, 0.0, 0.55],
            provenance,
            request_id: 44,
        };
        assert!(validate_plant_push(&nonfinite).is_err());

        let excessive_offset = PlantClientCommand::PlantPush {
            body: "base".to_owned(),
            force_world: [4.0, 0.0, 0.0],
            application_point_world: [0.0, 0.0, 1.301],
            provenance,
            request_id: 45,
        };
        assert!(validate_plant_application_point(&excessive_offset, &body_positions).is_err());
        let unknown_body = PlantClientCommand::PlantPush {
            body: "not_a_body".to_owned(),
            force_world: [4.0, 0.0, 0.0],
            application_point_world: [0.0, 0.0, 0.55],
            provenance,
            request_id: 46,
        };
        assert!(validate_plant_application_point(&unknown_body, &body_positions).is_err());

        let release = serde_json::from_str::<PlantClientCommand>(
            r#"{"type":"plant_release","request_id":47}"#,
        )
        .expect("typed plant release parses");
        assert_eq!(release, PlantClientCommand::PlantRelease { request_id: 47 });

        let pause =
            serde_json::from_str::<PlantClientCommand>(r#"{"type":"plant_pause","request_id":49}"#)
                .expect("typed plant pause parses");
        assert_eq!(pause, PlantClientCommand::PlantPause { request_id: 49 });
        let resume = serde_json::from_str::<PlantClientCommand>(
            r#"{"type":"plant_resume","request_id":50}"#,
        )
        .expect("typed plant resume parses");
        assert_eq!(resume, PlantClientCommand::PlantResume { request_id: 50 });

        for load_class in [
            PlantExternalLoadClass::MeasuredImpactImpulse,
            PlantExternalLoadClass::UnobservedModelReserve,
        ] {
            let non_command = PlantClientCommand::PlantPush {
                body: "base".to_owned(),
                force_world: [1.0, 0.0, 0.0],
                application_point_world: [0.0, 0.0, 0.55],
                provenance: PlantExternalLoadProvenance {
                    load_class,
                    ..provenance
                },
                request_id: 48,
            };
            assert!(validate_plant_push(&non_command).is_err());
        }
    }

    #[test]
    fn upkie_exposes_base_as_an_unclamped_cartesian_target() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/upkie/upkie.urdf");
        let program = MotionProgram::compile_urdf_file(path, TimingSpec::default(), 1).unwrap();
        let mut state = RobotState::zeros(&program.model);
        state.q = standing_posture(&program.model);
        state.control_world_from_root.translation.vector.z +=
            ground_root_lift(&program.model, &state).unwrap() + LIVE_GROUND_REGISTRATION_MARGIN_M;
        let mut squat = SquatContext::compile(&program.model, &state).unwrap();
        let handles = collect_interaction_handles(&program.model, Some(&squat));
        assert_eq!(
            handles
                .iter()
                .filter(|handle| handle.kind == "base")
                .count(),
            1
        );
        assert_eq!(
            handles
                .iter()
                .filter(|handle| handle.kind == "joint")
                .count(),
            4
        );
        assert_eq!(
            handles
                .iter()
                .map(|handle| handle.frame.as_str())
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            handles.len()
        );

        let requested = squat.standing_frame + Vec3::new(0.07, -0.03, -0.50);
        squat.update_target(requested);
        assert!(!squat.target_clamped);
        assert_eq!(
            squat.desired_root,
            squat.standing_root + Vec3::new(0.07, -0.03, -0.50)
        );
        assert_eq!(squat.target_clamp_error_m, 0.0);
        squat.reset();
        assert!(!squat.target_clamped);
        assert_eq!(squat.target_clamp_error_m, 0.0);
        squat.update_target(squat.standing_frame + Vec3::new(0.0, 0.0, -0.05));
        squat.release();
        assert!(!squat.engaged);
        assert!(!squat.target_clamped);

        let targets = compile_upkie_planar_ik_targets(&program.model, &squat).unwrap();
        let mut target = state.clone();
        let mut scratch = PlanarIkScratch::new(&program.model);
        let mut cache = ModelCache::new(&program.model);
        let requested_root = squat.standing_root + Vec3::new(0.0, 0.0, -0.50);
        let (applied_root, clamp_error_m) = prepare_ground_safe_balanced_upkie_squat_target(
            &program.model,
            &state,
            &standing_posture(&program.model),
            squat.standing_root,
            requested_root,
            &targets,
            &squat.support_targets,
            &mut target,
            &mut scratch,
            &mut cache,
        )
        .unwrap();
        assert!(clamp_error_m > 0.0);
        assert!(applied_root.z > requested_root.z);
        assert!(
            ground_root_lift(&program.model, &target).unwrap()
                <= 1.1e-6 - LIVE_GROUND_REGISTRATION_MARGIN_M
        );
    }

    #[test]
    fn upkie_authority_contract_is_explicit_and_non_aggregating() {
        let contract = AuthorityContract::editor(false);
        assert_eq!(contract.schema, 1);
        let ids = contract
            .signals
            .iter()
            .map(|signal| signal.stable_id)
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(ids.len(), contract.signals.len());
        assert_eq!(contract.signals.len(), 23);
        assert_eq!(
            contract
                .signals
                .iter()
                .find(|signal| signal.stable_id == "finite_support")
                .unwrap()
                .availability,
            AuthorityAvailability::Unavailable
        );
        assert_eq!(
            contract
                .signals
                .iter()
                .find(|signal| signal.stable_id == "joint_stopping")
                .unwrap()
                .availability,
            AuthorityAvailability::Measured
        );
        for stable_id in [
            "self_collision_avoidance",
            "world_collision_avoidance",
            "world_scene_snapshot",
            "robot_observation_history",
            "robot_observation_authority",
            "command_tracking_authority",
            "command_sampled_geometry",
            "command_continuous_clearance",
            "command_world_sampled_geometry",
            "command_root_prediction",
            "command_root_prediction_error",
            "command_world_continuous_clearance",
            "command_selection",
        ] {
            assert_eq!(
                contract
                    .signals
                    .iter()
                    .find(|signal| signal.stable_id == stable_id)
                    .unwrap()
                    .availability,
                AuthorityAvailability::Measured
            );
        }
        assert_eq!(
            contract
                .signals
                .iter()
                .find(|signal| signal.stable_id == "task_residuals")
                .unwrap()
                .availability,
            AuthorityAvailability::Measured
        );
        assert!(
            contract
                .signals
                .iter()
                .all(|signal| signal.stable_id != "health")
        );
    }

    #[test]
    fn observation_transport_modes_have_deterministic_query_and_delivery_contracts() {
        assert_eq!(
            ObservationTransportMode::Exact.query_time_ns(20_000_000),
            20_000_000
        );
        assert_eq!(
            ObservationTransportMode::Interpolated.query_time_ns(20_000_000),
            10_000_000
        );
        assert!(ObservationTransportMode::Exact.emits_sample(0, false));
        assert!(ObservationTransportMode::Interpolated.emits_sample(0, false));
        assert!(!ObservationTransportMode::Predicted.emits_sample(20_000_000, false));
        assert!(!ObservationTransportMode::Predicted.emits_sample(60_000_000, false));
        assert!(ObservationTransportMode::Predicted.emits_sample(40_000_000, false));
        assert!(ObservationTransportMode::Predicted.emits_sample(0, true));
        assert!(!ObservationTransportMode::Stale.emits_sample(5_000_000, true));
    }

    #[test]
    fn interpolation_mode_seeds_a_bracket_when_selected_before_execution() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/toy_humanoid.urdf");
        let program = MotionProgram::compile_urdf_file(path, TimingSpec::default(), 118).unwrap();
        let mut state = FloatingRobotState::zeros(&program.model);
        state.robot.q = standing_posture(&program.model);
        let mut history = RobotObservationHistory::new(&program.model, 4);
        let mut report = RobotObservationIngestReport::default();
        assert!(
            ingest_live_observation(
                &mut history,
                &program.model,
                program.header.program_epoch,
                20_000_000,
                5,
                &state,
                ObservationTransportMode::Interpolated,
                &mut report,
            )
            .unwrap()
        );
        assert_eq!(report.accepted(), 2);
        assert_eq!(history.len(), 2);
        let mut output = FloatingRobotState::zeros(&program.model);
        let evidence = history
            .reconstruct_into(
                &program.model,
                17_500_000,
                LIVE_ROBOT_OBSERVATION_QUERY_POLICY,
                &mut output,
            )
            .unwrap();
        assert_eq!(evidence.provenance, ReconstructionProvenance::Interpolated);
        assert!(evidence.hard_constraint_eligible);
    }

    #[test]
    fn flat_foot_profile_derives_two_finite_patches_and_composes_stopping_bounds() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/toy_humanoid.urdf");
        let program = MotionProgram::compile_urdf_file(path, TimingSpec::default(), 1).unwrap();
        let mut state = RobotState::zeros(&program.model);
        state.q = standing_posture(&program.model);
        state.control_world_from_root.translation.vector.z +=
            ground_root_lift(&program.model, &state).unwrap();
        let squat = SquatContext::compile(&program.model, &state).unwrap();
        let total_weight = program
            .model
            .bodies
            .iter()
            .map(|body| body.mass)
            .sum::<f64>()
            * 9.81;
        let profile =
            compile_editor_contact_profile(&program.model, Some(&squat), None, total_weight)
                .unwrap();
        assert_eq!(profile.contacts.len(), 8);
        assert_eq!(profile.support_patches.len(), 2);
        assert_eq!(profile.render_support_patches.len(), 2);
        for patch in &profile.support_patches {
            assert_eq!(patch.contact_count, 4);
            assert_eq!(patch.minimum_margin_m, 0.02);
            let rows = profile.contacts
                [patch.first_contact..patch.first_contact + patch.contact_count]
                .iter()
                .map(|contact| {
                    if !contact.kinematic_enabled {
                        0
                    } else {
                        match contact.mode {
                            ContactMode::LockedPoint => 3,
                            ContactMode::NormalPoint => 1,
                            ContactMode::RollingPoint => 2,
                            ContactMode::RollingWheel { .. } => unreachable!(),
                        }
                    }
                })
                .sum::<usize>();
            assert_eq!(rows, 6);
        }
        let mut bounds = VelocityBounds::unbounded(program.model.dof + 6);
        let recovery = compose_joint_stopping_bounds(
            &program.model,
            &state,
            0.0,
            0.0,
            200.0,
            0.005,
            &mut bounds,
        )
        .unwrap();
        assert_eq!(recovery, 0);
        assert!(bounds.validate(program.model.dof + 6));
        let qdd = DVector::zeros(program.model.dof + 6);
        let (_, margin) = minimum_joint_stopping_margin(&qdd, &bounds, program.model.dof).unwrap();
        assert!(margin.is_finite() && margin > 0.0);

        let flat_contract = AuthorityContract::editor(true);
        assert_eq!(
            flat_contract
                .signals
                .iter()
                .find(|signal| signal.stable_id == "finite_support")
                .unwrap()
                .availability,
            AuthorityAvailability::Measured
        );
    }

    #[test]
    fn upkie_geometry_stream_is_ground_registered() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/upkie/upkie.urdf");
        let program = MotionProgram::compile_urdf_file(path, TimingSpec::default(), 1).unwrap();
        let mut state = RobotState::zeros(&program.model);
        state.q = standing_posture(&program.model);
        let lift = ground_root_lift(&program.model, &state).unwrap();
        assert!(lift > 0.0);
        state.control_world_from_root.translation.vector.z += lift;
        assert!(ground_root_lift(&program.model, &state).unwrap().abs() < 1e-12);

        let geometry = collect_render_geometry(&program.model);
        assert!(geometry.len() >= 20);
        assert!(
            geometry
                .iter()
                .any(|shape| matches!(shape, RenderGeometry::Box { .. }))
        );
        assert!(
            geometry
                .iter()
                .any(|shape| matches!(shape, RenderGeometry::Cylinder { .. }))
        );
        let visuals = collect_render_visuals(&program.model);
        assert_eq!(visuals.len(), 41);
        assert_eq!(
            visuals
                .iter()
                .filter(|visual| matches!(visual.shape, RenderGeometry::Mesh { .. }))
                .count(),
            25
        );

        let torque_bounds = actuator_torque_bounds(&program).unwrap();
        let mut output =
            FloatingDynamicWbcOutput::workspace(program.model.dof, 2, program.model.dof + 16);
        let knee = program.model.joints[program.model.joint_id("left_knee").unwrap().0]
            .coordinate
            .unwrap();
        output.actuator_torque[knee] = 0.9 * torque_bounds.upper[knee];
        let utilization =
            maximum_actuator_effort_utilization(&output.actuator_torque, &torque_bounds)
                .unwrap()
                .unwrap();
        assert_eq!(utilization.coordinate, knee);
        assert!((utilization.utilization - 0.9).abs() < 1e-12);

        let headroom = minimum_joint_position_headroom(&program.model, &state)
            .unwrap()
            .unwrap();
        assert!(headroom.margin_rad > 0.0);
        assert!(headroom.fraction_of_range > 0.0 && headroom.fraction_of_range <= 0.5);
    }
}
