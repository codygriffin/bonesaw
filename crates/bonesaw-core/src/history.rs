use std::collections::VecDeque;

use thiserror::Error;

use crate::{
    math::{ControlTime, Motion6, SpatialAcceleration6, SymmetricMat6, Transform3, Vec3},
    model::{CompiledModel, FloatingRobotState, JointKind, RobotState},
};

/// Caller-authored provenance for one robot-state observation. Source time is
/// retained in the producer clock domain; only mapped time participates in
/// control decisions. The core never reads or maps a clock.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RobotObservationStamp {
    pub source_time_ns: i64,
    pub mapped_time_ns: ControlTime,
    pub source_sequence: u64,
    pub source_id: u64,
    pub synchronization_uncertainty_ns: i64,
}

impl RobotObservationStamp {
    pub fn exact_at(time_ns: ControlTime, source_id: u64, source_sequence: u64) -> Self {
        Self {
            source_time_ns: time_ns,
            mapped_time_ns: time_ns,
            source_sequence,
            source_id,
            synchronization_uncertainty_ns: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RobotObservationLimits {
    pub maximum_age_ns: Option<i64>,
    pub maximum_synchronization_uncertainty_ns: Option<i64>,
}

impl RobotObservationLimits {
    pub fn validate(self) -> bool {
        self.maximum_age_ns.is_none_or(|value| value >= 0)
            && self
                .maximum_synchronization_uncertainty_ns
                .is_none_or(|value| value >= 0)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RobotObservationEvidence {
    pub stamp: RobotObservationStamp,
    pub age_ns: i64,
    pub age_headroom_ns: i64,
    pub synchronization_uncertainty_headroom_ns: i64,
    pub causal: bool,
    pub age_valid: bool,
    pub synchronization_valid: bool,
}

impl RobotObservationEvidence {
    pub fn is_valid(self) -> bool {
        self.causal && self.age_valid && self.synchronization_valid
    }
}

impl Default for RobotObservationEvidence {
    fn default() -> Self {
        Self {
            stamp: RobotObservationStamp::default(),
            age_ns: i64::MAX,
            age_headroom_ns: i64::MIN,
            synchronization_uncertainty_headroom_ns: i64::MIN,
            causal: false,
            age_valid: false,
            synchronization_valid: false,
        }
    }
}

pub fn evaluate_robot_observation(
    tick_time_ns: ControlTime,
    stamp: RobotObservationStamp,
    limits: RobotObservationLimits,
) -> RobotObservationEvidence {
    let age_ns = tick_time_ns.saturating_sub(stamp.mapped_time_ns);
    let causal = stamp.mapped_time_ns <= tick_time_ns;
    let age_headroom_ns = limits
        .maximum_age_ns
        .map_or(i64::MAX, |limit| limit.saturating_sub(age_ns));
    let age_valid = limits.maximum_age_ns.is_none_or(|limit| age_ns <= limit);
    let synchronization_uncertainty_headroom_ns = limits
        .maximum_synchronization_uncertainty_ns
        .map_or(i64::MAX, |limit| {
            limit.saturating_sub(stamp.synchronization_uncertainty_ns)
        });
    let synchronization_valid = stamp.synchronization_uncertainty_ns >= 0
        && limits
            .maximum_synchronization_uncertainty_ns
            .is_none_or(|limit| stamp.synchronization_uncertainty_ns <= limit);
    RobotObservationEvidence {
        stamp,
        age_ns,
        age_headroom_ns,
        synchronization_uncertainty_headroom_ns,
        causal,
        age_valid,
        synchronization_valid,
    }
}

#[derive(Clone, Debug)]
pub struct TimedRobotState {
    pub time_ns: ControlTime,
    pub sequence: u64,
    pub state: RobotState,
}

/// Fixed-capacity authoritative history. Online insertion never shifts old
/// samples: out-of-order observations are rejected and exact duplicates are
/// replaced only by a higher sequence number.
#[derive(Clone, Debug)]
pub struct RobotHistory {
    capacity: usize,
    samples: VecDeque<TimedRobotState>,
}

#[derive(Clone, Copy, Debug)]
pub struct RobotObservationRef<'a> {
    pub program_epoch: u64,
    pub stamp: RobotObservationStamp,
    pub state: &'a FloatingRobotState,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RobotObservationIngestReport {
    pub batch_sorted: bool,
    pub policy_valid: bool,
    pub inserted: usize,
    pub replaced_sequence: usize,
    pub replaced_source: usize,
    pub ignored_duplicate: usize,
    pub ignored_source: usize,
    pub rejected_epoch: usize,
    pub rejected_future: usize,
    pub rejected_stale: usize,
    pub rejected_uncertain: usize,
    pub rejected_invalid_policy: usize,
    pub rejected_invalid_state: usize,
    pub rejected_out_of_order: usize,
    pub rejected_unsorted: usize,
}

impl RobotObservationIngestReport {
    pub fn accepted(self) -> usize {
        self.inserted + self.replaced_sequence + self.replaced_source
    }

    pub fn rejected(self) -> usize {
        self.rejected_epoch
            + self.rejected_future
            + self.rejected_stale
            + self.rejected_uncertain
            + self.rejected_invalid_policy
            + self.rejected_invalid_state
            + self.rejected_out_of_order
            + self.rejected_unsorted
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RobotObservationIngestDisposition {
    Inserted,
    ReplacedSequence,
    ReplacedSource,
    IgnoredDuplicate,
    IgnoredSource,
    RejectedEpoch,
    RejectedFuture,
    RejectedStale,
    RejectedUncertain,
    RejectedInvalidState,
    RejectedOutOfOrder,
}

#[derive(Clone, Debug)]
struct StoredRobotObservation {
    program_epoch: u64,
    stamp: RobotObservationStamp,
    state: FloatingRobotState,
}

/// Fixed-capacity canonical robot-observation history. Slots and generalized
/// state vectors are allocated once at construction; online ingest only copies
/// into those slots. For equal mapped timestamps, the lowest stable source ID
/// wins independent of arrival/chunking. Within that source, highest sequence
/// wins.
#[derive(Clone, Debug)]
pub struct RobotObservationHistory {
    slots: Vec<StoredRobotObservation>,
    head: usize,
    len: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RobotObservationQueryPolicy {
    pub maximum_interpolation_gap_ns: i64,
    pub maximum_extrapolation_ns: i64,
    pub maximum_source_age_ns: i64,
    pub maximum_synchronization_uncertainty_ns: i64,
    pub allow_prediction: bool,
    pub allow_hold: bool,
}

impl Default for RobotObservationQueryPolicy {
    fn default() -> Self {
        Self {
            maximum_interpolation_gap_ns: 100_000_000,
            maximum_extrapolation_ns: 40_000_000,
            maximum_source_age_ns: 40_000_000,
            maximum_synchronization_uncertainty_ns: 2_000_000,
            allow_prediction: true,
            allow_hold: false,
        }
    }
}

impl RobotObservationQueryPolicy {
    pub fn validate(self) -> bool {
        self.maximum_interpolation_gap_ns >= 0
            && self.maximum_extrapolation_ns >= 0
            && self.maximum_source_age_ns >= 0
            && self.maximum_synchronization_uncertainty_ns >= 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RobotObservationReconstructionEvidence {
    pub program_epoch: u64,
    pub provenance: ReconstructionProvenance,
    pub source_interval_ns: (i64, i64),
    pub lower_stamp: RobotObservationStamp,
    pub upper_stamp: RobotObservationStamp,
    pub source_age_ns: i64,
    pub source_age_headroom_ns: i64,
    pub maximum_synchronization_uncertainty_ns: i64,
    pub synchronization_headroom_ns: i64,
    pub hard_constraint_eligible: bool,
}

impl RobotObservationReconstructionEvidence {
    /// Mapped control time represented by the reconstructed state. This is
    /// derived from the lower source and the conservative source-age witness,
    /// so callers do not need to retain an untyped parallel query timestamp.
    pub fn query_time_ns(self) -> ControlTime {
        self.lower_stamp
            .mapped_time_ns
            .saturating_add(self.source_age_ns)
    }

    /// Stamp suitable for a downstream admission check on the reconstructed
    /// state. Source identity/sequence and clock projection follow the upper
    /// contributor, while mapped time is the state time actually queried and
    /// synchronization uncertainty conservatively covers both contributors.
    pub fn reconstructed_stamp(self) -> RobotObservationStamp {
        let query_time_ns = self.query_time_ns();
        RobotObservationStamp {
            source_time_ns: self
                .upper_stamp
                .source_time_ns
                .saturating_add(query_time_ns.saturating_sub(self.upper_stamp.mapped_time_ns)),
            mapped_time_ns: query_time_ns,
            source_sequence: self.upper_stamp.source_sequence,
            source_id: self.upper_stamp.source_id,
            synchronization_uncertainty_ns: self.maximum_synchronization_uncertainty_ns,
        }
    }

    /// Conservative state/model error exposure for this reconstruction. Exact
    /// samples expose only clock synchronization uncertainty. Interpolation
    /// uses distance to the nearest bracket endpoint; prediction/hold uses the
    /// forward horizon from the newest contributor. This keeps endpoint error
    /// continuous and makes prediction uncertainty monotone in horizon.
    pub fn conservative_error_bound(
        self,
        growth: RobotObservationErrorGrowth,
    ) -> Result<RobotObservationErrorBound, RobotObservationErrorError> {
        if !growth.validate() || self.maximum_synchronization_uncertainty_ns < 0 {
            return Err(RobotObservationErrorError::InvalidGrowthOrEvidence);
        }
        let query_time_ns = self.query_time_ns();
        let model_exposure_ns = match self.provenance {
            ReconstructionProvenance::ExactSample => 0,
            ReconstructionProvenance::Interpolated => query_time_ns
                .saturating_sub(self.lower_stamp.mapped_time_ns)
                .min(
                    self.upper_stamp
                        .mapped_time_ns
                        .saturating_sub(query_time_ns),
                )
                .max(0),
            ReconstructionProvenance::PredictedConstantVelocity
            | ReconstructionProvenance::Held => query_time_ns
                .saturating_sub(self.upper_stamp.mapped_time_ns)
                .max(0),
        };
        let exposure_ns =
            model_exposure_ns.saturating_add(self.maximum_synchronization_uncertainty_ns);
        let seconds = exposure_ns as f64 * 1e-9;
        let position_growth = |initial: f64, velocity: f64, acceleration: f64| {
            initial + velocity * seconds + 0.5 * acceleration * seconds * seconds
        };
        let bound = RobotObservationErrorBound {
            exposure_ns,
            joint_position_error_rad: position_growth(
                growth.initial_joint_position_error_rad,
                growth.joint_velocity_error_bound_rad_s,
                growth.joint_acceleration_error_bound_rad_s2,
            ),
            joint_velocity_error_rad_s: growth.initial_joint_velocity_error_rad_s
                + growth.joint_acceleration_error_bound_rad_s2 * seconds,
            root_translation_error_m: position_growth(
                growth.initial_root_translation_error_m,
                growth.root_linear_velocity_error_bound_m_s,
                growth.root_linear_acceleration_error_bound_m_s2,
            ),
            root_rotation_error_rad: position_growth(
                growth.initial_root_rotation_error_rad,
                growth.root_angular_velocity_error_bound_rad_s,
                growth.root_angular_acceleration_error_bound_rad_s2,
            ),
            represented_point_position_error_m: position_growth(
                growth.initial_represented_point_position_error_m,
                growth.represented_point_velocity_error_bound_m_s,
                growth.represented_point_acceleration_error_bound_m_s2,
            ),
            center_of_mass_position_error_m: position_growth(
                growth.initial_center_of_mass_position_error_m,
                growth.center_of_mass_velocity_error_bound_m_s,
                growth.center_of_mass_acceleration_error_bound_m_s2,
            ),
        };
        if !bound.validate() {
            return Err(RobotObservationErrorError::InvalidGrowthOrEvidence);
        }
        Ok(bound)
    }
}

/// Caller-authored conservative error-growth envelope. These are deterministic
/// bounds, not covariance, confidence intervals, or a calibrated estimator
/// claim. Position-like errors use `initial + velocity*t + 0.5*acceleration*t²`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RobotObservationErrorGrowth {
    pub initial_joint_position_error_rad: f64,
    pub initial_joint_velocity_error_rad_s: f64,
    pub joint_velocity_error_bound_rad_s: f64,
    pub joint_acceleration_error_bound_rad_s2: f64,
    pub initial_root_translation_error_m: f64,
    pub root_linear_velocity_error_bound_m_s: f64,
    pub root_linear_acceleration_error_bound_m_s2: f64,
    pub initial_root_rotation_error_rad: f64,
    pub root_angular_velocity_error_bound_rad_s: f64,
    pub root_angular_acceleration_error_bound_rad_s2: f64,
    pub initial_represented_point_position_error_m: f64,
    pub represented_point_velocity_error_bound_m_s: f64,
    pub represented_point_acceleration_error_bound_m_s2: f64,
    pub initial_center_of_mass_position_error_m: f64,
    pub center_of_mass_velocity_error_bound_m_s: f64,
    pub center_of_mass_acceleration_error_bound_m_s2: f64,
}

impl RobotObservationErrorGrowth {
    pub fn validate(self) -> bool {
        [
            self.initial_joint_position_error_rad,
            self.initial_joint_velocity_error_rad_s,
            self.joint_velocity_error_bound_rad_s,
            self.joint_acceleration_error_bound_rad_s2,
            self.initial_root_translation_error_m,
            self.root_linear_velocity_error_bound_m_s,
            self.root_linear_acceleration_error_bound_m_s2,
            self.initial_root_rotation_error_rad,
            self.root_angular_velocity_error_bound_rad_s,
            self.root_angular_acceleration_error_bound_rad_s2,
            self.initial_represented_point_position_error_m,
            self.represented_point_velocity_error_bound_m_s,
            self.represented_point_acceleration_error_bound_m_s2,
            self.initial_center_of_mass_position_error_m,
            self.center_of_mass_velocity_error_bound_m_s,
            self.center_of_mass_acceleration_error_bound_m_s2,
        ]
        .into_iter()
        .all(|value| value.is_finite() && value >= 0.0)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RobotObservationErrorBound {
    pub exposure_ns: i64,
    pub joint_position_error_rad: f64,
    pub joint_velocity_error_rad_s: f64,
    pub root_translation_error_m: f64,
    pub root_rotation_error_rad: f64,
    pub represented_point_position_error_m: f64,
    pub center_of_mass_position_error_m: f64,
}

impl RobotObservationErrorBound {
    pub fn validate(self) -> bool {
        self.exposure_ns >= 0
            && [
                self.joint_position_error_rad,
                self.joint_velocity_error_rad_s,
                self.root_translation_error_m,
                self.root_rotation_error_rad,
                self.represented_point_position_error_m,
                self.center_of_mass_position_error_m,
            ]
            .into_iter()
            .all(|value| value.is_finite() && value >= 0.0)
    }
}

#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum RobotObservationErrorError {
    #[error("robot observation error growth or reconstruction evidence is invalid")]
    InvalidGrowthOrEvidence,
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum RobotObservationQueryError {
    #[error("robot observation history is empty")]
    Empty,
    #[error("requested time is older than retained robot observation history")]
    BeforeHistory,
    #[error("robot observation interpolation gap exceeds policy")]
    InterpolationGap,
    #[error("robot observation prediction is disabled or exceeds its horizon")]
    Extrapolation,
    #[error("newest robot observation exceeds the maximum source age")]
    TooOld,
    #[error("robot observation synchronization uncertainty exceeds policy")]
    SynchronizationUncertain,
    #[error("reconstructed robot observation is outside the compiled state manifold")]
    ReconstructedStateInvalid,
    #[error("robot observation query policy or output layout is invalid")]
    InvalidPolicyOrLayout,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HistoryInsert {
    Inserted,
    Replaced,
    IgnoredDuplicate,
    RejectedOutOfOrder,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReconstructionProvenance {
    ExactSample,
    Interpolated,
    PredictedConstantVelocity,
    Held,
}

#[derive(Clone, Copy, Debug)]
pub struct HistoryQueryPolicy {
    pub maximum_interpolation_gap_ns: i64,
    pub maximum_extrapolation_ns: i64,
    pub allow_prediction: bool,
    pub allow_hold: bool,
}

impl Default for HistoryQueryPolicy {
    fn default() -> Self {
        Self {
            maximum_interpolation_gap_ns: 100_000_000,
            maximum_extrapolation_ns: 40_000_000,
            allow_prediction: true,
            allow_hold: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ReconstructedState {
    pub state: RobotState,
    pub provenance: ReconstructionProvenance,
    pub source_interval_ns: (i64, i64),
}

/// Allocation-free evidence returned when reconstruction writes into a
/// caller-owned [`RobotState`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReconstructedStateEvidence {
    pub provenance: ReconstructionProvenance,
    pub source_interval_ns: (i64, i64),
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum HistoryQueryError {
    #[error("history is empty")]
    Empty,
    #[error("requested time is older than retained history")]
    BeforeHistory,
    #[error("interpolation gap exceeds policy")]
    InterpolationGap,
    #[error("prediction is disabled or exceeds its configured horizon")]
    Extrapolation,
    #[error("reconstruction output has q={q} and v={v}, expected {expected}")]
    OutputLayout { expected: usize, q: usize, v: usize },
    #[error("stored robot state has q={q} and v={v}, expected {expected}")]
    StateLayout { expected: usize, q: usize, v: usize },
}

#[derive(Clone, Debug)]
pub struct ExternalFrameSample {
    pub time_ns: ControlTime,
    pub anchor_from_frame: Transform3,
    /// Spatial velocity ordered `[angular; linear]`, expressed in the anchor.
    pub twist: Option<Motion6>,
    pub acceleration: Option<SpatialAcceleration6>,
    pub covariance: Option<SymmetricMat6>,
    pub sequence: u64,
}

impl ExternalFrameSample {
    /// Identity sample used to construct caller-owned reconstruction storage.
    /// The value is overwritten before a successful reconstruction returns.
    pub fn workspace() -> Self {
        Self {
            time_ns: 0,
            anchor_from_frame: Transform3::identity(),
            twist: None,
            acceleration: None,
            covariance: None,
            sequence: 0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ReconstructedExternalFrame {
    pub sample: ExternalFrameSample,
    pub provenance: ReconstructionProvenance,
    pub source_interval_ns: (i64, i64),
}

/// One fixed-capacity external-frame slot. It uses the same deterministic
/// duplicate, ordering, interpolation-gap, and extrapolation policies as the
/// canonical robot history.
#[derive(Clone, Debug)]
pub struct ExternalFrameHistory {
    capacity: usize,
    samples: VecDeque<ExternalFrameSample>,
}

impl ExternalFrameHistory {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0);
        Self {
            capacity,
            samples: VecDeque::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, sample: ExternalFrameSample) -> HistoryInsert {
        if let Some(latest) = self.samples.back_mut() {
            if sample.time_ns < latest.time_ns {
                return HistoryInsert::RejectedOutOfOrder;
            }
            if sample.time_ns == latest.time_ns {
                if sample.sequence > latest.sequence {
                    *latest = sample;
                    return HistoryInsert::Replaced;
                }
                return HistoryInsert::IgnoredDuplicate;
            }
        }
        if self.samples.len() == self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
        HistoryInsert::Inserted
    }

    pub fn reconstruct(
        &self,
        time_ns: ControlTime,
        policy: HistoryQueryPolicy,
    ) -> Result<ReconstructedExternalFrame, HistoryQueryError> {
        let first = self.samples.front().ok_or(HistoryQueryError::Empty)?;
        let last = self.samples.back().expect("non-empty history has a back");
        if time_ns < first.time_ns {
            return Err(HistoryQueryError::BeforeHistory);
        }
        if time_ns == last.time_ns {
            return Ok(reconstructed_external(
                last.clone(),
                ReconstructionProvenance::ExactSample,
                (last.time_ns, last.time_ns),
            ));
        }
        if time_ns > last.time_ns {
            let horizon_ns = time_ns - last.time_ns;
            if horizon_ns > policy.maximum_extrapolation_ns {
                return Err(HistoryQueryError::Extrapolation);
            }
            if policy.allow_prediction
                && let Some(twist) = last.twist
            {
                let dt = horizon_ns as f64 * 1e-9;
                let angular = Vec3::new(twist.0[0], twist.0[1], twist.0[2]);
                let linear = Vec3::new(twist.0[3], twist.0[4], twist.0[5]);
                let mut predicted = last.clone();
                predicted.time_ns = time_ns;
                predicted.anchor_from_frame = Transform3::from_parts(
                    nalgebra::Translation3::from(
                        last.anchor_from_frame.translation.vector + linear * dt,
                    ),
                    nalgebra::UnitQuaternion::from_scaled_axis(angular * dt)
                        * last.anchor_from_frame.rotation,
                );
                return Ok(reconstructed_external(
                    predicted,
                    ReconstructionProvenance::PredictedConstantVelocity,
                    (last.time_ns, time_ns),
                ));
            }
            if policy.allow_hold {
                let mut held = last.clone();
                held.time_ns = time_ns;
                return Ok(reconstructed_external(
                    held,
                    ReconstructionProvenance::Held,
                    (last.time_ns, time_ns),
                ));
            }
            return Err(HistoryQueryError::Extrapolation);
        }

        let (lower, upper) = self
            .samples
            .iter()
            .zip(self.samples.iter().skip(1))
            .find(|(lower, upper)| lower.time_ns <= time_ns && time_ns <= upper.time_ns)
            .expect("time within retained external history has a bracket");
        if time_ns == lower.time_ns {
            return Ok(reconstructed_external(
                lower.clone(),
                ReconstructionProvenance::ExactSample,
                (lower.time_ns, lower.time_ns),
            ));
        }
        if time_ns == upper.time_ns {
            return Ok(reconstructed_external(
                upper.clone(),
                ReconstructionProvenance::ExactSample,
                (upper.time_ns, upper.time_ns),
            ));
        }
        let gap = upper.time_ns - lower.time_ns;
        if gap > policy.maximum_interpolation_gap_ns {
            return Err(HistoryQueryError::InterpolationGap);
        }
        let alpha = (time_ns - lower.time_ns) as f64 / gap as f64;
        let interpolate_motion = |a: Option<Motion6>, b: Option<Motion6>| match (a, b) {
            (Some(a), Some(b)) => Some(Motion6((1.0 - alpha) * a.0 + alpha * b.0)),
            _ => None,
        };
        let interpolate_acceleration =
            |a: Option<SpatialAcceleration6>, b: Option<SpatialAcceleration6>| match (a, b) {
                (Some(a), Some(b)) => Some(SpatialAcceleration6((1.0 - alpha) * a.0 + alpha * b.0)),
                _ => None,
            };
        let covariance = match (&lower.covariance, &upper.covariance) {
            (Some(a), Some(b)) => Some((1.0 - alpha) * a + alpha * b),
            _ => None,
        };
        let sample = ExternalFrameSample {
            time_ns,
            anchor_from_frame: Transform3::from_parts(
                nalgebra::Translation3::from(
                    (1.0 - alpha) * lower.anchor_from_frame.translation.vector
                        + alpha * upper.anchor_from_frame.translation.vector,
                ),
                lower
                    .anchor_from_frame
                    .rotation
                    .slerp(&upper.anchor_from_frame.rotation, alpha),
            ),
            twist: interpolate_motion(lower.twist, upper.twist),
            acceleration: interpolate_acceleration(lower.acceleration, upper.acceleration),
            covariance,
            sequence: lower.sequence.max(upper.sequence),
        };
        Ok(reconstructed_external(
            sample,
            ReconstructionProvenance::Interpolated,
            (lower.time_ns, upper.time_ns),
        ))
    }

    /// Reconstruct into caller-owned sample storage. This is the hot-path
    /// companion to [`Self::reconstruct`]; it preserves the same interpolation,
    /// prediction, hold, and interval semantics without cloning a sample.
    pub fn reconstruct_into(
        &self,
        time_ns: ControlTime,
        policy: HistoryQueryPolicy,
        output: &mut ExternalFrameSample,
    ) -> Result<ReconstructedStateEvidence, HistoryQueryError> {
        let first = self.samples.front().ok_or(HistoryQueryError::Empty)?;
        let last = self.samples.back().expect("non-empty history has a back");
        if time_ns < first.time_ns {
            return Err(HistoryQueryError::BeforeHistory);
        }
        if time_ns == last.time_ns {
            copy_external_sample_into(output, last);
            return Ok(ReconstructedStateEvidence {
                provenance: ReconstructionProvenance::ExactSample,
                source_interval_ns: (last.time_ns, last.time_ns),
            });
        }
        if time_ns > last.time_ns {
            let horizon_ns = time_ns - last.time_ns;
            if horizon_ns > policy.maximum_extrapolation_ns {
                return Err(HistoryQueryError::Extrapolation);
            }
            if policy.allow_prediction
                && let Some(twist) = last.twist
            {
                copy_external_sample_into(output, last);
                let dt = horizon_ns as f64 * 1e-9;
                let angular = Vec3::new(twist.0[0], twist.0[1], twist.0[2]);
                let linear = Vec3::new(twist.0[3], twist.0[4], twist.0[5]);
                output.time_ns = time_ns;
                output.anchor_from_frame = Transform3::from_parts(
                    nalgebra::Translation3::from(
                        last.anchor_from_frame.translation.vector + linear * dt,
                    ),
                    nalgebra::UnitQuaternion::from_scaled_axis(angular * dt)
                        * last.anchor_from_frame.rotation,
                );
                return Ok(ReconstructedStateEvidence {
                    provenance: ReconstructionProvenance::PredictedConstantVelocity,
                    source_interval_ns: (last.time_ns, time_ns),
                });
            }
            if policy.allow_hold {
                copy_external_sample_into(output, last);
                output.time_ns = time_ns;
                return Ok(ReconstructedStateEvidence {
                    provenance: ReconstructionProvenance::Held,
                    source_interval_ns: (last.time_ns, time_ns),
                });
            }
            return Err(HistoryQueryError::Extrapolation);
        }

        let (lower, upper) = self
            .samples
            .iter()
            .zip(self.samples.iter().skip(1))
            .find(|(lower, upper)| lower.time_ns <= time_ns && time_ns <= upper.time_ns)
            .expect("time within retained external history has a bracket");
        if time_ns == lower.time_ns {
            copy_external_sample_into(output, lower);
            return Ok(ReconstructedStateEvidence {
                provenance: ReconstructionProvenance::ExactSample,
                source_interval_ns: (lower.time_ns, lower.time_ns),
            });
        }
        if time_ns == upper.time_ns {
            copy_external_sample_into(output, upper);
            return Ok(ReconstructedStateEvidence {
                provenance: ReconstructionProvenance::ExactSample,
                source_interval_ns: (upper.time_ns, upper.time_ns),
            });
        }
        let gap = upper.time_ns - lower.time_ns;
        if gap > policy.maximum_interpolation_gap_ns {
            return Err(HistoryQueryError::InterpolationGap);
        }
        let alpha = (time_ns - lower.time_ns) as f64 / gap as f64;
        output.time_ns = time_ns;
        output.anchor_from_frame = Transform3::from_parts(
            nalgebra::Translation3::from(
                (1.0 - alpha) * lower.anchor_from_frame.translation.vector
                    + alpha * upper.anchor_from_frame.translation.vector,
            ),
            lower
                .anchor_from_frame
                .rotation
                .slerp(&upper.anchor_from_frame.rotation, alpha),
        );
        output.twist = match (lower.twist, upper.twist) {
            (Some(a), Some(b)) => Some(Motion6((1.0 - alpha) * a.0 + alpha * b.0)),
            _ => None,
        };
        output.acceleration = match (lower.acceleration, upper.acceleration) {
            (Some(a), Some(b)) => Some(SpatialAcceleration6((1.0 - alpha) * a.0 + alpha * b.0)),
            _ => None,
        };
        output.covariance = match (lower.covariance, upper.covariance) {
            (Some(a), Some(b)) => Some((1.0 - alpha) * a + alpha * b),
            _ => None,
        };
        output.sequence = lower.sequence.max(upper.sequence);
        Ok(ReconstructedStateEvidence {
            provenance: ReconstructionProvenance::Interpolated,
            source_interval_ns: (lower.time_ns, upper.time_ns),
        })
    }
}

fn copy_external_sample_into(destination: &mut ExternalFrameSample, source: &ExternalFrameSample) {
    destination.time_ns = source.time_ns;
    destination.anchor_from_frame = source.anchor_from_frame;
    destination.twist = source.twist;
    destination.acceleration = source.acceleration;
    destination.covariance = source.covariance;
    destination.sequence = source.sequence;
}

fn reconstructed_external(
    sample: ExternalFrameSample,
    provenance: ReconstructionProvenance,
    source_interval_ns: (i64, i64),
) -> ReconstructedExternalFrame {
    ReconstructedExternalFrame {
        sample,
        provenance,
        source_interval_ns,
    }
}

#[derive(Clone, Debug)]
pub struct ExternalFrameHistories {
    slots: Vec<ExternalFrameHistory>,
}

impl ExternalFrameHistories {
    pub fn new(capacities: impl IntoIterator<Item = usize>) -> Self {
        Self {
            slots: capacities
                .into_iter()
                .map(ExternalFrameHistory::new)
                .collect(),
        }
    }

    pub fn slot(&self, index: usize) -> Option<&ExternalFrameHistory> {
        self.slots.get(index)
    }

    pub fn slot_mut(&mut self, index: usize) -> Option<&mut ExternalFrameHistory> {
        self.slots.get_mut(index)
    }

    pub fn len(&self) -> usize {
        self.slots.len()
    }

    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }
}

impl RobotObservationHistory {
    pub fn new(model: &CompiledModel, capacity: usize) -> Self {
        assert!(capacity > 0);
        let slots = (0..capacity)
            .map(|_| StoredRobotObservation {
                program_epoch: 0,
                stamp: RobotObservationStamp::default(),
                state: FloatingRobotState::zeros(model),
            })
            .collect();
        Self {
            slots,
            head: 0,
            len: 0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.slots.len()
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    pub fn latest_stamp(&self) -> Option<RobotObservationStamp> {
        self.latest().map(|sample| sample.stamp)
    }

    fn physical_index(&self, logical_index: usize) -> usize {
        (self.head + logical_index) % self.capacity()
    }

    fn sample(&self, logical_index: usize) -> &StoredRobotObservation {
        &self.slots[self.physical_index(logical_index)]
    }

    fn latest(&self) -> Option<&StoredRobotObservation> {
        (self.len > 0).then(|| self.sample(self.len - 1))
    }

    fn latest_index(&self) -> Option<usize> {
        (self.len > 0).then(|| self.physical_index(self.len - 1))
    }

    pub fn ingest_batch(
        &mut self,
        model: &CompiledModel,
        tick_time_ns: ControlTime,
        expected_program_epoch: u64,
        limits: RobotObservationLimits,
        observations: &[RobotObservationRef<'_>],
    ) -> RobotObservationIngestReport {
        let mut report = RobotObservationIngestReport {
            batch_sorted: true,
            policy_valid: true,
            ..RobotObservationIngestReport::default()
        };
        if !limits.validate() {
            report.policy_valid = false;
            report.rejected_invalid_policy = observations.len();
            return report;
        }
        if !batch_is_sorted(observations) {
            report.batch_sorted = false;
            report.rejected_unsorted = observations.len();
            return report;
        }
        for observation in observations {
            let disposition = self.ingest_one(
                model,
                tick_time_ns,
                expected_program_epoch,
                limits,
                *observation,
            );
            match disposition {
                RobotObservationIngestDisposition::Inserted => report.inserted += 1,
                RobotObservationIngestDisposition::ReplacedSequence => {
                    report.replaced_sequence += 1
                }
                RobotObservationIngestDisposition::ReplacedSource => report.replaced_source += 1,
                RobotObservationIngestDisposition::IgnoredDuplicate => {
                    report.ignored_duplicate += 1
                }
                RobotObservationIngestDisposition::IgnoredSource => report.ignored_source += 1,
                RobotObservationIngestDisposition::RejectedEpoch => report.rejected_epoch += 1,
                RobotObservationIngestDisposition::RejectedFuture => report.rejected_future += 1,
                RobotObservationIngestDisposition::RejectedStale => report.rejected_stale += 1,
                RobotObservationIngestDisposition::RejectedUncertain => {
                    report.rejected_uncertain += 1
                }
                RobotObservationIngestDisposition::RejectedInvalidState => {
                    report.rejected_invalid_state += 1
                }
                RobotObservationIngestDisposition::RejectedOutOfOrder => {
                    report.rejected_out_of_order += 1
                }
            }
        }
        report
    }

    pub fn ingest_one(
        &mut self,
        model: &CompiledModel,
        tick_time_ns: ControlTime,
        expected_program_epoch: u64,
        limits: RobotObservationLimits,
        observation: RobotObservationRef<'_>,
    ) -> RobotObservationIngestDisposition {
        if observation.program_epoch != expected_program_epoch {
            return RobotObservationIngestDisposition::RejectedEpoch;
        }
        let timing = evaluate_robot_observation(tick_time_ns, observation.stamp, limits);
        if !timing.causal {
            return RobotObservationIngestDisposition::RejectedFuture;
        }
        if !timing.age_valid {
            return RobotObservationIngestDisposition::RejectedStale;
        }
        if !timing.synchronization_valid {
            return RobotObservationIngestDisposition::RejectedUncertain;
        }
        if observation.state.validate(model).is_err() {
            return RobotObservationIngestDisposition::RejectedInvalidState;
        }
        if let Some(latest_index) = self.latest_index() {
            let latest = &self.slots[latest_index];
            if observation.stamp.mapped_time_ns < latest.stamp.mapped_time_ns {
                return RobotObservationIngestDisposition::RejectedOutOfOrder;
            }
            if observation.stamp.mapped_time_ns == latest.stamp.mapped_time_ns {
                if observation.stamp.source_id < latest.stamp.source_id {
                    copy_observation_into(&mut self.slots[latest_index], observation);
                    return RobotObservationIngestDisposition::ReplacedSource;
                }
                if observation.stamp.source_id > latest.stamp.source_id {
                    return RobotObservationIngestDisposition::IgnoredSource;
                }
                if observation.stamp.source_sequence > latest.stamp.source_sequence {
                    copy_observation_into(&mut self.slots[latest_index], observation);
                    return RobotObservationIngestDisposition::ReplacedSequence;
                }
                return RobotObservationIngestDisposition::IgnoredDuplicate;
            }
        }

        let destination = if self.len < self.capacity() {
            let index = self.physical_index(self.len);
            self.len += 1;
            index
        } else {
            let index = self.head;
            self.head = (self.head + 1) % self.capacity();
            index
        };
        copy_observation_into(&mut self.slots[destination], observation);
        RobotObservationIngestDisposition::Inserted
    }

    pub fn reconstruct_into(
        &self,
        model: &CompiledModel,
        time_ns: ControlTime,
        policy: RobotObservationQueryPolicy,
        output: &mut FloatingRobotState,
    ) -> Result<RobotObservationReconstructionEvidence, RobotObservationQueryError> {
        if !policy.validate()
            || output.robot.q.len() != model.dof
            || output.robot.v.len() != model.dof
        {
            return Err(RobotObservationQueryError::InvalidPolicyOrLayout);
        }
        let first = (self.len > 0)
            .then(|| self.sample(0))
            .ok_or(RobotObservationQueryError::Empty)?;
        let last = self
            .latest()
            .expect("non-empty observation history has latest");
        if time_ns < first.stamp.mapped_time_ns {
            return Err(RobotObservationQueryError::BeforeHistory);
        }
        if time_ns == last.stamp.mapped_time_ns {
            copy_floating_robot_state_into(output, &last.state);
            return reconstruction_evidence(
                time_ns,
                last,
                last,
                ReconstructionProvenance::ExactSample,
                policy,
            );
        }
        if time_ns > last.stamp.mapped_time_ns {
            let horizon_ns = time_ns.saturating_sub(last.stamp.mapped_time_ns);
            if horizon_ns > policy.maximum_source_age_ns {
                return Err(RobotObservationQueryError::TooOld);
            }
            if horizon_ns > policy.maximum_extrapolation_ns {
                return Err(RobotObservationQueryError::Extrapolation);
            }
            copy_floating_robot_state_into(output, &last.state);
            let provenance = if policy.allow_prediction {
                predict_floating_robot_state_into(model, horizon_ns, output);
                ReconstructionProvenance::PredictedConstantVelocity
            } else if policy.allow_hold {
                ReconstructionProvenance::Held
            } else {
                return Err(RobotObservationQueryError::Extrapolation);
            };
            return reconstruction_evidence(time_ns, last, last, provenance, policy);
        }

        let mut bracket = None;
        for logical in 0..self.len - 1 {
            let lower = self.sample(logical);
            let upper = self.sample(logical + 1);
            if lower.stamp.mapped_time_ns <= time_ns && time_ns <= upper.stamp.mapped_time_ns {
                bracket = Some((lower, upper));
                break;
            }
        }
        let (lower, upper) = bracket.expect("time within retained history has bracket");
        if time_ns == lower.stamp.mapped_time_ns {
            copy_floating_robot_state_into(output, &lower.state);
            return reconstruction_evidence(
                time_ns,
                lower,
                lower,
                ReconstructionProvenance::ExactSample,
                policy,
            );
        }
        if time_ns == upper.stamp.mapped_time_ns {
            copy_floating_robot_state_into(output, &upper.state);
            return reconstruction_evidence(
                time_ns,
                upper,
                upper,
                ReconstructionProvenance::ExactSample,
                policy,
            );
        }
        let gap_ns = upper.stamp.mapped_time_ns - lower.stamp.mapped_time_ns;
        if gap_ns > policy.maximum_interpolation_gap_ns {
            return Err(RobotObservationQueryError::InterpolationGap);
        }
        interpolate_floating_robot_state_into(model, lower, upper, time_ns, output);
        if output.validate(model).is_err() {
            return Err(RobotObservationQueryError::ReconstructedStateInvalid);
        }
        reconstruction_evidence(
            time_ns,
            lower,
            upper,
            ReconstructionProvenance::Interpolated,
            policy,
        )
    }
}

fn batch_is_sorted(observations: &[RobotObservationRef<'_>]) -> bool {
    observations.windows(2).all(|pair| {
        let left = pair[0].stamp;
        let right = pair[1].stamp;
        (left.mapped_time_ns, left.source_id) <= (right.mapped_time_ns, right.source_id)
    })
}

fn copy_robot_state_into(destination: &mut RobotState, source: &RobotState) {
    destination.control_world_from_root = source.control_world_from_root;
    destination
        .q
        .as_mut_slice()
        .copy_from_slice(source.q.as_slice());
    destination
        .v
        .as_mut_slice()
        .copy_from_slice(source.v.as_slice());
}

fn copy_floating_robot_state_into(
    destination: &mut FloatingRobotState,
    source: &FloatingRobotState,
) {
    copy_robot_state_into(&mut destination.robot, &source.robot);
    destination.root_twist_world = source.root_twist_world;
}

fn copy_observation_into(
    destination: &mut StoredRobotObservation,
    source: RobotObservationRef<'_>,
) {
    destination.program_epoch = source.program_epoch;
    destination.stamp = source.stamp;
    copy_floating_robot_state_into(&mut destination.state, source.state);
}

fn interpolate_floating_robot_state_into(
    model: &CompiledModel,
    lower: &StoredRobotObservation,
    upper: &StoredRobotObservation,
    time_ns: ControlTime,
    output: &mut FloatingRobotState,
) {
    let gap_ns = upper.stamp.mapped_time_ns - lower.stamp.mapped_time_ns;
    let duration_seconds = gap_ns as f64 * 1e-9;
    let alpha = (time_ns - lower.stamp.mapped_time_ns) as f64 / gap_ns as f64;
    let alpha_squared = alpha * alpha;
    let alpha_cubed = alpha_squared * alpha;
    let h10 = alpha_cubed - 2.0 * alpha_squared + alpha;
    let h01 = -2.0 * alpha_cubed + 3.0 * alpha_squared;
    let h11 = alpha_cubed - alpha_squared;
    let dh10 = 3.0 * alpha_squared - 4.0 * alpha + 1.0;
    let dh01 = -6.0 * alpha_squared + 6.0 * alpha;
    let dh11 = 3.0 * alpha_squared - 2.0 * alpha;

    let lower_pose = lower.state.robot.control_world_from_root;
    let upper_pose = upper.state.robot.control_world_from_root;
    let translation_delta = upper_pose.translation.vector - lower_pose.translation.vector;
    let lower_linear = lower
        .state
        .root_twist_world
        .0
        .fixed_rows::<3>(3)
        .into_owned();
    let upper_linear = upper
        .state
        .root_twist_world
        .0
        .fixed_rows::<3>(3)
        .into_owned();
    let translation = lower_pose.translation.vector
        + h10 * duration_seconds * lower_linear
        + h01 * translation_delta
        + h11 * duration_seconds * upper_linear;

    let rotation_delta = (upper_pose.rotation * lower_pose.rotation.inverse()).scaled_axis();
    let lower_angular = lower
        .state
        .root_twist_world
        .0
        .fixed_rows::<3>(0)
        .into_owned();
    let upper_angular = upper
        .state
        .root_twist_world
        .0
        .fixed_rows::<3>(0)
        .into_owned();
    let rotation_tangent = h10 * duration_seconds * lower_angular
        + h01 * rotation_delta
        + h11 * duration_seconds * upper_angular;
    output.robot.control_world_from_root = Transform3::from_parts(
        nalgebra::Translation3::from(translation),
        nalgebra::UnitQuaternion::from_scaled_axis(rotation_tangent) * lower_pose.rotation,
    );
    let linear_velocity =
        dh10 * lower_linear + (dh01 / duration_seconds) * translation_delta + dh11 * upper_linear;
    let angular_velocity =
        dh10 * lower_angular + (dh01 / duration_seconds) * rotation_delta + dh11 * upper_angular;
    output
        .root_twist_world
        .0
        .fixed_rows_mut::<3>(0)
        .copy_from(&angular_velocity);
    output
        .root_twist_world
        .0
        .fixed_rows_mut::<3>(3)
        .copy_from(&linear_velocity);

    for joint in &model.joints {
        let Some(index) = joint.coordinate else {
            continue;
        };
        let mut difference = upper.state.robot.q[index] - lower.state.robot.q[index];
        if joint.kind == JointKind::Continuous {
            difference = (difference + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                - std::f64::consts::PI;
        }
        let lower_velocity = lower.state.robot.v[index];
        let upper_velocity = upper.state.robot.v[index];
        let mut position = lower.state.robot.q[index]
            + h10 * duration_seconds * lower_velocity
            + h01 * difference
            + h11 * duration_seconds * upper_velocity;
        if joint.kind == JointKind::Continuous {
            position = (position + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                - std::f64::consts::PI;
        }
        output.robot.q[index] = position;
        output.robot.v[index] =
            dh10 * lower_velocity + (dh01 / duration_seconds) * difference + dh11 * upper_velocity;
    }
}

fn predict_floating_robot_state_into(
    model: &CompiledModel,
    horizon_ns: i64,
    output: &mut FloatingRobotState,
) {
    let dt = horizon_ns as f64 * 1e-9;
    let angular_step = Vec3::new(
        output.root_twist_world.0[0],
        output.root_twist_world.0[1],
        output.root_twist_world.0[2],
    ) * dt;
    let linear_step = Vec3::new(
        output.root_twist_world.0[3],
        output.root_twist_world.0[4],
        output.root_twist_world.0[5],
    ) * dt;
    output.robot.control_world_from_root.rotation =
        nalgebra::UnitQuaternion::from_scaled_axis(angular_step)
            * output.robot.control_world_from_root.rotation;
    output.robot.control_world_from_root.translation.vector += linear_step;
    for joint in &model.joints {
        let Some(index) = joint.coordinate else {
            continue;
        };
        let mut position = output.robot.q[index] + output.robot.v[index] * dt;
        if joint.kind == JointKind::Continuous {
            position = (position + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                - std::f64::consts::PI;
        } else {
            position = position.clamp(joint.limit.lower, joint.limit.upper);
            if (position <= joint.limit.lower && output.robot.v[index] < 0.0)
                || (position >= joint.limit.upper && output.robot.v[index] > 0.0)
            {
                output.robot.v[index] = 0.0;
            }
        }
        output.robot.q[index] = position;
    }
}

fn reconstruction_evidence(
    time_ns: ControlTime,
    lower: &StoredRobotObservation,
    upper: &StoredRobotObservation,
    provenance: ReconstructionProvenance,
    policy: RobotObservationQueryPolicy,
) -> Result<RobotObservationReconstructionEvidence, RobotObservationQueryError> {
    let source_age_ns = time_ns.saturating_sub(lower.stamp.mapped_time_ns);
    let source_age_headroom_ns = policy.maximum_source_age_ns.saturating_sub(source_age_ns);
    let maximum_synchronization_uncertainty_ns = lower
        .stamp
        .synchronization_uncertainty_ns
        .max(upper.stamp.synchronization_uncertainty_ns);
    let synchronization_headroom_ns = policy
        .maximum_synchronization_uncertainty_ns
        .saturating_sub(maximum_synchronization_uncertainty_ns);
    if synchronization_headroom_ns < 0 || maximum_synchronization_uncertainty_ns < 0 {
        return Err(RobotObservationQueryError::SynchronizationUncertain);
    }
    Ok(RobotObservationReconstructionEvidence {
        program_epoch: lower.program_epoch,
        provenance,
        source_interval_ns: (lower.stamp.mapped_time_ns, upper.stamp.mapped_time_ns),
        lower_stamp: lower.stamp,
        upper_stamp: upper.stamp,
        source_age_ns,
        source_age_headroom_ns,
        maximum_synchronization_uncertainty_ns,
        synchronization_headroom_ns,
        hard_constraint_eligible: provenance != ReconstructionProvenance::Held
            && source_age_headroom_ns >= 0
            && synchronization_headroom_ns >= 0,
    })
}

impl RobotHistory {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0);
        Self {
            capacity,
            samples: VecDeque::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, sample: TimedRobotState) -> HistoryInsert {
        if let Some(latest) = self.samples.back_mut() {
            if sample.time_ns < latest.time_ns {
                return HistoryInsert::RejectedOutOfOrder;
            }
            if sample.time_ns == latest.time_ns {
                if sample.sequence > latest.sequence {
                    *latest = sample;
                    return HistoryInsert::Replaced;
                }
                return HistoryInsert::IgnoredDuplicate;
            }
        }
        if self.samples.len() == self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
        HistoryInsert::Inserted
    }

    pub fn latest(&self) -> Option<&TimedRobotState> {
        self.samples.back()
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn reconstruct(
        &self,
        model: &CompiledModel,
        time_ns: ControlTime,
        policy: HistoryQueryPolicy,
    ) -> Result<ReconstructedState, HistoryQueryError> {
        let mut state = RobotState::zeros(model);
        let evidence = self.reconstruct_into(model, time_ns, policy, &mut state)?;
        Ok(ReconstructedState {
            state,
            provenance: evidence.provenance,
            source_interval_ns: evidence.source_interval_ns,
        })
    }

    /// Reconstruct into caller-owned storage. Once `output` has the model's
    /// fixed layout this path does not allocate, including interpolation and
    /// constant-velocity prediction.
    pub fn reconstruct_into(
        &self,
        model: &CompiledModel,
        time_ns: ControlTime,
        policy: HistoryQueryPolicy,
        output: &mut RobotState,
    ) -> Result<ReconstructedStateEvidence, HistoryQueryError> {
        if output.q.len() != model.dof || output.v.len() != model.dof {
            return Err(HistoryQueryError::OutputLayout {
                expected: model.dof,
                q: output.q.len(),
                v: output.v.len(),
            });
        }
        let first = self.samples.front().ok_or(HistoryQueryError::Empty)?;
        let last = self.samples.back().expect("non-empty history has a back");
        if first.state.q.len() != model.dof
            || first.state.v.len() != model.dof
            || last.state.q.len() != model.dof
            || last.state.v.len() != model.dof
        {
            return Err(HistoryQueryError::StateLayout {
                expected: model.dof,
                q: first.state.q.len().max(last.state.q.len()),
                v: first.state.v.len().max(last.state.v.len()),
            });
        }
        if time_ns < first.time_ns {
            return Err(HistoryQueryError::BeforeHistory);
        }
        if time_ns == last.time_ns {
            copy_robot_state_into(output, &last.state);
            return Ok(ReconstructedStateEvidence {
                provenance: ReconstructionProvenance::ExactSample,
                source_interval_ns: (last.time_ns, last.time_ns),
            });
        }
        if time_ns > last.time_ns {
            let horizon = time_ns - last.time_ns;
            if policy.allow_prediction && horizon <= policy.maximum_extrapolation_ns {
                copy_robot_state_into(output, &last.state);
                model.integrate(output, &last.state.v, horizon as f64 * 1e-9);
                return Ok(ReconstructedStateEvidence {
                    provenance: ReconstructionProvenance::PredictedConstantVelocity,
                    source_interval_ns: (last.time_ns, time_ns),
                });
            }
            if policy.allow_hold && horizon <= policy.maximum_extrapolation_ns {
                copy_robot_state_into(output, &last.state);
                return Ok(ReconstructedStateEvidence {
                    provenance: ReconstructionProvenance::Held,
                    source_interval_ns: (last.time_ns, time_ns),
                });
            }
            return Err(HistoryQueryError::Extrapolation);
        }

        let (lower, upper) = self
            .samples
            .iter()
            .zip(self.samples.iter().skip(1))
            .find(|(lower, upper)| lower.time_ns <= time_ns && time_ns <= upper.time_ns)
            .expect("time within retained history has a bracket");
        if lower.state.q.len() != model.dof
            || lower.state.v.len() != model.dof
            || upper.state.q.len() != model.dof
            || upper.state.v.len() != model.dof
        {
            return Err(HistoryQueryError::StateLayout {
                expected: model.dof,
                q: lower.state.q.len().max(upper.state.q.len()),
                v: lower.state.v.len().max(upper.state.v.len()),
            });
        }
        if time_ns == lower.time_ns {
            copy_robot_state_into(output, &lower.state);
            return Ok(ReconstructedStateEvidence {
                provenance: ReconstructionProvenance::ExactSample,
                source_interval_ns: (lower.time_ns, lower.time_ns),
            });
        }
        if time_ns == upper.time_ns {
            copy_robot_state_into(output, &upper.state);
            return Ok(ReconstructedStateEvidence {
                provenance: ReconstructionProvenance::ExactSample,
                source_interval_ns: (upper.time_ns, upper.time_ns),
            });
        }
        let gap = upper.time_ns - lower.time_ns;
        if gap > policy.maximum_interpolation_gap_ns {
            return Err(HistoryQueryError::InterpolationGap);
        }
        let alpha = (time_ns - lower.time_ns) as f64 / gap as f64;
        let control_world_from_root = crate::math::Transform3::from_parts(
            nalgebra::Translation3::from(
                (1.0 - alpha) * lower.state.control_world_from_root.translation.vector
                    + alpha * upper.state.control_world_from_root.translation.vector,
            ),
            lower
                .state
                .control_world_from_root
                .rotation
                .slerp(&upper.state.control_world_from_root.rotation, alpha),
        );
        output.control_world_from_root = control_world_from_root;
        for index in 0..model.dof {
            output.q[index] = lower.state.q[index];
            output.v[index] = (1.0 - alpha) * lower.state.v[index] + alpha * upper.state.v[index];
        }
        for joint in &model.joints {
            let Some(index) = joint.coordinate else {
                continue;
            };
            let mut difference = upper.state.q[index] - lower.state.q[index];
            if joint.kind == JointKind::Continuous {
                difference = (difference + std::f64::consts::PI)
                    .rem_euclid(2.0 * std::f64::consts::PI)
                    - std::f64::consts::PI;
            }
            output.q[index] = lower.state.q[index] + alpha * difference;
        }
        Ok(ReconstructedStateEvidence {
            provenance: ReconstructionProvenance::Interpolated,
            source_interval_ns: (lower.time_ns, upper.time_ns),
        })
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{DVector, Matrix3};

    use super::*;
    use crate::{
        math::Vec3,
        model::{BodyId, FrameId, JointId, JointLimit, JointSpec, RigidBodySpec},
    };

    fn sample(time_ns: i64, sequence: u64) -> TimedRobotState {
        TimedRobotState {
            time_ns,
            sequence,
            state: RobotState {
                control_world_from_root: crate::math::Transform3::identity(),
                q: DVector::from_element(1, time_ns as f64),
                v: DVector::zeros(1),
            },
        }
    }

    #[test]
    fn deterministic_duplicate_and_wrap_policy() {
        let mut history = RobotHistory::new(2);
        assert_eq!(history.push(sample(1, 1)), HistoryInsert::Inserted);
        assert_eq!(history.push(sample(1, 0)), HistoryInsert::IgnoredDuplicate);
        assert_eq!(history.push(sample(1, 2)), HistoryInsert::Replaced);
        assert_eq!(history.push(sample(2, 1)), HistoryInsert::Inserted);
        assert_eq!(history.push(sample(3, 1)), HistoryInsert::Inserted);
        assert_eq!(history.len(), 2);
        assert_eq!(
            history.push(sample(2, 9)),
            HistoryInsert::RejectedOutOfOrder
        );
    }

    fn continuous_model() -> CompiledModel {
        let body = |id, name: &str| RigidBodySpec {
            id: BodyId(id),
            name: name.into(),
            parent_joint: None,
            body_frame: FrameId(id),
            mass: 1.0,
            com_in_body: Vec3::zeros(),
            inertia_about_com_in_body: Matrix3::identity(),
            collisions: vec![],
            visuals: vec![],
        };
        CompiledModel::new(
            "continuous".into(),
            vec![body(0, "base"), body(1, "tip")],
            vec![JointSpec {
                id: JointId(0),
                name: "spin".into(),
                kind: JointKind::Continuous,
                parent: BodyId(0),
                child: BodyId(1),
                parent_from_joint: nalgebra::Isometry3::identity(),
                axis_in_joint: Vec3::z(),
                coordinate: None,
                limit: JointLimit::unbounded(),
            }],
        )
        .unwrap()
    }

    fn observation_state(
        model: &CompiledModel,
        position: f64,
        velocity: f64,
    ) -> FloatingRobotState {
        let mut state = FloatingRobotState::zeros(model);
        state.robot.q[0] = position;
        state.robot.v[0] = velocity;
        state
    }

    fn observation_stamp(time_ns: i64, source_id: u64, sequence: u64) -> RobotObservationStamp {
        RobotObservationStamp {
            source_time_ns: time_ns - 100,
            mapped_time_ns: time_ns,
            source_sequence: sequence,
            source_id,
            synchronization_uncertainty_ns: 200,
        }
    }

    #[test]
    fn observation_batch_authority_is_chunking_invariant_and_wraps_without_growth() {
        let model = continuous_model();
        let limits = RobotObservationLimits {
            maximum_age_ns: Some(100),
            maximum_synchronization_uncertainty_ns: Some(1_000),
        };
        let states = [
            observation_state(&model, 0.1, 0.0),
            observation_state(&model, 0.2, 0.0),
            observation_state(&model, 0.3, 0.0),
            observation_state(&model, 0.9, 0.0),
            observation_state(&model, 1.0, 0.0),
        ];
        let stamps = [
            observation_stamp(10, 9, 1),
            observation_stamp(10, 9, 2),
            observation_stamp(20, 3, 1),
            observation_stamp(20, 9, 1),
            observation_stamp(30, 3, 2),
        ];
        let whole_refs: Vec<_> = states[..4]
            .iter()
            .zip(stamps[..4].iter())
            .map(|(state, stamp)| RobotObservationRef {
                program_epoch: 116,
                stamp: *stamp,
                state,
            })
            .collect();
        let mut whole = RobotObservationHistory::new(&model, 2);
        let report = whole.ingest_batch(&model, 20, 116, limits, &whole_refs);
        assert_eq!(report.inserted, 2);
        assert_eq!(report.replaced_sequence, 1);
        assert_eq!(report.ignored_source, 1);

        let mut chunked = RobotObservationHistory::new(&model, 2);
        let first_refs = [
            RobotObservationRef {
                program_epoch: 116,
                stamp: stamps[0],
                state: &states[0],
            },
            RobotObservationRef {
                program_epoch: 116,
                stamp: stamps[1],
                state: &states[1],
            },
        ];
        chunked.ingest_batch(&model, 20, 116, limits, &first_refs);
        chunked.ingest_one(
            &model,
            20,
            116,
            limits,
            RobotObservationRef {
                program_epoch: 116,
                stamp: stamps[3],
                state: &states[3],
            },
        );
        assert_eq!(
            chunked.ingest_one(
                &model,
                20,
                116,
                limits,
                RobotObservationRef {
                    program_epoch: 116,
                    stamp: stamps[2],
                    state: &states[2],
                },
            ),
            RobotObservationIngestDisposition::ReplacedSource
        );

        let policy = RobotObservationQueryPolicy::default();
        let mut whole_out = FloatingRobotState::zeros(&model);
        let mut chunked_out = FloatingRobotState::zeros(&model);
        let whole_evidence = whole
            .reconstruct_into(&model, 20, policy, &mut whole_out)
            .unwrap();
        let chunked_evidence = chunked
            .reconstruct_into(&model, 20, policy, &mut chunked_out)
            .unwrap();
        assert_eq!(whole_out.robot.q, chunked_out.robot.q);
        assert_eq!(whole_out.robot.q[0], 0.3);
        assert_eq!(whole_evidence, chunked_evidence);
        assert_eq!(whole_evidence.lower_stamp.source_id, 3);
        assert_eq!(whole_evidence.program_epoch, 116);

        let allocation = RobotObservationRef {
            program_epoch: 116,
            stamp: stamps[4],
            state: &states[4],
        };
        whole.ingest_batch(&model, 30, 116, limits, &[allocation]);
        assert_eq!(whole.len(), 2);
        assert_eq!(whole.capacity(), 2);
        assert_eq!(
            whole.reconstruct_into(&model, 10, policy, &mut whole_out),
            Err(RobotObservationQueryError::BeforeHistory)
        );
    }

    #[test]
    fn observation_ingest_rejects_faults_and_unsorted_batches_atomically() {
        let model = continuous_model();
        let good = observation_state(&model, 0.0, 0.0);
        let mut invalid = good.clone();
        invalid.robot.q[0] = f64::NAN;
        let limits = RobotObservationLimits {
            maximum_age_ns: Some(10),
            maximum_synchronization_uncertainty_ns: Some(1_000),
        };
        let late = RobotObservationRef {
            program_epoch: 116,
            stamp: observation_stamp(20, 1, 1),
            state: &good,
        };
        let early = RobotObservationRef {
            program_epoch: 116,
            stamp: observation_stamp(10, 1, 1),
            state: &good,
        };
        let mut history = RobotObservationHistory::new(&model, 4);
        let report = history.ingest_batch(&model, 20, 116, limits, &[late, early]);
        assert!(!report.batch_sorted);
        assert_eq!(report.rejected_unsorted, 2);
        assert!(history.is_empty());

        let invalid_policy = RobotObservationLimits {
            maximum_age_ns: Some(-1),
            maximum_synchronization_uncertainty_ns: Some(1_000),
        };
        let report = history.ingest_batch(&model, 20, 116, invalid_policy, &[late]);
        assert!(!report.policy_valid);
        assert_eq!(report.rejected_invalid_policy, 1);
        assert!(history.is_empty());

        let cases = [
            RobotObservationRef {
                program_epoch: 115,
                stamp: observation_stamp(20, 1, 1),
                state: &good,
            },
            RobotObservationRef {
                program_epoch: 116,
                stamp: observation_stamp(21, 2, 1),
                state: &good,
            },
            RobotObservationRef {
                program_epoch: 116,
                stamp: observation_stamp(5, 3, 1),
                state: &good,
            },
            RobotObservationRef {
                program_epoch: 116,
                stamp: RobotObservationStamp {
                    synchronization_uncertainty_ns: 1_001,
                    ..observation_stamp(20, 4, 1)
                },
                state: &good,
            },
            RobotObservationRef {
                program_epoch: 116,
                stamp: observation_stamp(20, 5, 1),
                state: &invalid,
            },
        ];
        assert_eq!(
            history.ingest_one(&model, 20, 116, limits, cases[0]),
            RobotObservationIngestDisposition::RejectedEpoch
        );
        assert_eq!(
            history.ingest_one(&model, 20, 116, limits, cases[1]),
            RobotObservationIngestDisposition::RejectedFuture
        );
        assert_eq!(
            history.ingest_one(&model, 20, 116, limits, cases[2]),
            RobotObservationIngestDisposition::RejectedStale
        );
        assert_eq!(
            history.ingest_one(&model, 20, 116, limits, cases[3]),
            RobotObservationIngestDisposition::RejectedUncertain
        );
        assert_eq!(
            history.ingest_one(&model, 20, 116, limits, cases[4]),
            RobotObservationIngestDisposition::RejectedInvalidState
        );
        assert!(history.is_empty());
    }

    #[test]
    fn observation_reconstruction_exposes_provenance_and_hard_constraint_eligibility() {
        let model = continuous_model();
        let mut states = [
            observation_state(&model, 0.0, 1.0),
            observation_state(&model, 0.02, 1.0),
        ];
        states[1].robot.control_world_from_root.translation.x = 0.02;
        states[0].root_twist_world.0[3] = 1.0;
        states[1].root_twist_world.0[3] = 1.0;
        let observations = [
            RobotObservationRef {
                program_epoch: 116,
                stamp: observation_stamp(0, 2, 1),
                state: &states[0],
            },
            RobotObservationRef {
                program_epoch: 116,
                stamp: observation_stamp(20_000_000, 2, 2),
                state: &states[1],
            },
        ];
        let mut history = RobotObservationHistory::new(&model, 4);
        history.ingest_batch(
            &model,
            20_000_000,
            116,
            RobotObservationLimits {
                maximum_age_ns: Some(30_000_000),
                maximum_synchronization_uncertainty_ns: Some(1_000),
            },
            &observations,
        );
        let mut output = FloatingRobotState::zeros(&model);
        let policy = RobotObservationQueryPolicy {
            maximum_source_age_ns: 30_000_000,
            maximum_synchronization_uncertainty_ns: 1_000,
            ..RobotObservationQueryPolicy::default()
        };
        let interpolated = history
            .reconstruct_into(&model, 10_000_000, policy, &mut output)
            .unwrap();
        assert_eq!(
            interpolated.provenance,
            ReconstructionProvenance::Interpolated
        );
        assert!((output.robot.q[0] - 0.01).abs() < 1e-12);
        assert!((output.robot.control_world_from_root.translation.x - 0.01).abs() < 1e-12);
        assert!((output.root_twist_world.0[3] - 1.0).abs() < 1e-12);
        assert!(interpolated.hard_constraint_eligible);
        assert_eq!(interpolated.query_time_ns(), 10_000_000);
        assert_eq!(
            interpolated.reconstructed_stamp().mapped_time_ns,
            10_000_000
        );
        assert_eq!(interpolated.reconstructed_stamp().source_time_ns, 9_999_900);
        assert_eq!(interpolated.reconstructed_stamp().source_sequence, 2);

        let predicted = history
            .reconstruct_into(&model, 25_000_000, policy, &mut output)
            .unwrap();
        assert_eq!(
            predicted.provenance,
            ReconstructionProvenance::PredictedConstantVelocity
        );
        assert!((output.robot.q[0] - 0.025).abs() < 1e-12);
        assert!((output.robot.control_world_from_root.translation.x - 0.025).abs() < 1e-12);
        assert!(predicted.hard_constraint_eligible);
        assert_eq!(predicted.query_time_ns(), 25_000_000);
        assert_eq!(predicted.reconstructed_stamp().mapped_time_ns, 25_000_000);
        assert_eq!(predicted.reconstructed_stamp().source_time_ns, 24_999_900);

        let growth = RobotObservationErrorGrowth {
            joint_velocity_error_bound_rad_s: 1.0,
            joint_acceleration_error_bound_rad_s2: 2.0,
            represented_point_velocity_error_bound_m_s: 0.5,
            center_of_mass_velocity_error_bound_m_s: 0.25,
            ..RobotObservationErrorGrowth::default()
        };
        let exact = history
            .reconstruct_into(&model, 20_000_000, policy, &mut output)
            .unwrap()
            .conservative_error_bound(growth)
            .unwrap();
        let predicted_five_ms = predicted.conservative_error_bound(growth).unwrap();
        let predicted_ten_ms = history
            .reconstruct_into(&model, 30_000_000, policy, &mut output)
            .unwrap()
            .conservative_error_bound(growth)
            .unwrap();
        assert_eq!(exact.exposure_ns, 200);
        assert_eq!(predicted_five_ms.exposure_ns, 5_000_200);
        assert_eq!(predicted_ten_ms.exposure_ns, 10_000_200);
        assert!(exact.joint_position_error_rad < predicted_five_ms.joint_position_error_rad);
        assert!(
            predicted_five_ms.joint_position_error_rad < predicted_ten_ms.joint_position_error_rad
        );
        assert!(
            predicted_five_ms.represented_point_position_error_m
                < predicted_ten_ms.represented_point_position_error_m
        );

        let held = history
            .reconstruct_into(
                &model,
                25_000_000,
                RobotObservationQueryPolicy {
                    allow_prediction: false,
                    allow_hold: true,
                    ..policy
                },
                &mut output,
            )
            .unwrap();
        assert_eq!(held.provenance, ReconstructionProvenance::Held);
        assert!(!held.hard_constraint_eligible);
        assert_eq!(
            history.reconstruct_into(&model, 51_000_000, policy, &mut output),
            Err(RobotObservationQueryError::TooOld)
        );
    }

    #[test]
    fn interpolation_uses_short_continuous_arc_and_prediction_is_bounded() {
        let model = continuous_model();
        let mut history = RobotHistory::new(4);
        let mut a = sample(0, 1);
        a.state.q[0] = 3.0;
        a.state.v[0] = 1.0;
        let mut b = sample(20_000_000, 1);
        b.state.q[0] = -3.0;
        b.state.v[0] = 1.0;
        history.push(a);
        history.push(b);
        let policy = HistoryQueryPolicy::default();
        let middle = history.reconstruct(&model, 10_000_000, policy).unwrap();
        assert_eq!(middle.provenance, ReconstructionProvenance::Interpolated);
        assert!(middle.state.q[0].abs() > 3.0);
        let mut middle_into = RobotState::zeros(&model);
        let middle_evidence = history
            .reconstruct_into(&model, 10_000_000, policy, &mut middle_into)
            .unwrap();
        assert_eq!(middle_evidence.provenance, middle.provenance);
        assert_eq!(
            middle_evidence.source_interval_ns,
            middle.source_interval_ns
        );
        assert_eq!(
            middle_into.control_world_from_root,
            middle.state.control_world_from_root
        );
        assert_eq!(middle_into.q, middle.state.q);
        assert_eq!(middle_into.v, middle.state.v);
        let future = history.reconstruct(&model, 40_000_000, policy).unwrap();
        assert_eq!(
            future.provenance,
            ReconstructionProvenance::PredictedConstantVelocity
        );
        assert!(matches!(
            history.reconstruct(&model, 100_000_000, policy),
            Err(HistoryQueryError::Extrapolation)
        ));
        let mut wrong_layout = RobotState {
            control_world_from_root: Transform3::identity(),
            q: nalgebra::DVector::zeros(0),
            v: nalgebra::DVector::zeros(0),
        };
        assert_eq!(
            history.reconstruct_into(&model, 10_000_000, policy, &mut wrong_layout),
            Err(HistoryQueryError::OutputLayout {
                expected: 1,
                q: 0,
                v: 0,
            })
        );
        let mut malformed = RobotHistory::new(2);
        malformed.push(TimedRobotState {
            time_ns: 0,
            sequence: 1,
            state: wrong_layout,
        });
        let mut valid_output = RobotState::zeros(&model);
        assert_eq!(
            malformed.reconstruct_into(&model, 0, policy, &mut valid_output),
            Err(HistoryQueryError::StateLayout {
                expected: 1,
                q: 0,
                v: 0,
            })
        );
    }

    #[test]
    fn external_pose_interpolates_and_predicts_with_bounded_twist() {
        let mut history = ExternalFrameHistory::new(4);
        let twist = Motion6(nalgebra::SVector::<f64, 6>::new(
            0.0, 0.0, 1.0, 1.0, 0.0, 0.0,
        ));
        for (time_ns, x) in [(0, 0.0), (20_000_000, 0.02)] {
            assert_eq!(
                history.push(ExternalFrameSample {
                    time_ns,
                    anchor_from_frame: Transform3::translation(x, 0.0, 0.0),
                    twist: Some(twist),
                    acceleration: None,
                    covariance: None,
                    sequence: 1,
                }),
                HistoryInsert::Inserted
            );
        }
        let policy = HistoryQueryPolicy::default();
        let middle = history.reconstruct(10_000_000, policy).unwrap();
        assert_eq!(middle.provenance, ReconstructionProvenance::Interpolated);
        assert!((middle.sample.anchor_from_frame.translation.x - 0.01).abs() < 1e-12);
        let mut middle_into = ExternalFrameSample::workspace();
        let middle_evidence = history
            .reconstruct_into(10_000_000, policy, &mut middle_into)
            .unwrap();
        assert_eq!(middle_evidence.provenance, middle.provenance);
        assert_eq!(
            middle_evidence.source_interval_ns,
            middle.source_interval_ns
        );
        assert_eq!(middle_into.time_ns, middle.sample.time_ns);
        assert_eq!(
            middle_into.anchor_from_frame,
            middle.sample.anchor_from_frame
        );
        assert_eq!(middle_into.twist, middle.sample.twist);
        assert_eq!(middle_into.acceleration, middle.sample.acceleration);
        assert_eq!(middle_into.covariance, middle.sample.covariance);
        assert_eq!(middle_into.sequence, middle.sample.sequence);

        let future = history.reconstruct(40_000_000, policy).unwrap();
        assert_eq!(
            future.provenance,
            ReconstructionProvenance::PredictedConstantVelocity
        );
        assert!((future.sample.anchor_from_frame.translation.x - 0.04).abs() < 1e-12);
        let future_evidence = history
            .reconstruct_into(40_000_000, policy, &mut middle_into)
            .unwrap();
        assert_eq!(future_evidence.provenance, future.provenance);
        assert_eq!(
            future_evidence.source_interval_ns,
            future.source_interval_ns
        );
        assert_eq!(
            middle_into.anchor_from_frame,
            future.sample.anchor_from_frame
        );
        assert!(
            future
                .sample
                .anchor_from_frame
                .rotation
                .angle_to(&nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.02))
                < 1e-12
        );
    }
}
