use nalgebra::{DMatrix, DVector, Point3, Vector2};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    actuation::CompiledActuation,
    collision::{
        CollisionAccelerationBarrierConfig, CollisionAccelerationBarrierEvidence,
        CollisionAccelerationBarrierScratch, CollisionError, CompiledCollisionModel,
    },
    history::RobotObservationErrorBound,
    math::{Motion6, Vec3},
    model::{CompiledModel, DynamicsCache, FrameId, ModelCache, ModelError, RobotState},
    solver::{
        ConstraintBuffer, HierarchicalSolver, Priority, SolveDiagnostics, SolveResult, SolveStatus,
        SolverWorkspace, Task, TaskKind, VelocityBounds,
    },
    world_collision::{
        CompiledWorldCollisionModel, WorldCollisionBarrierEvidence, WorldCollisionBarrierScratch,
        WorldCollisionError,
    },
};

const DYNAMICS_ROW_BASE: u32 = 0x1000_0000;
const CONTACT_ROW_BASE: u32 = 0x2000_0000;
const FRICTION_ROW_BASE: u32 = 0x3000_0000;
const SUPPORT_ROW_BASE: u32 = 0x4000_0000;
const ACTUATOR_EFFORT_ROW_BASE: u32 = 0x5000_0000;
const SUPPORT_PATCH_POINT_CAPACITY: usize = 16;
pub const FLOATING_POINT_TASK_CAPACITY: usize = 4;
/// Fixed slots for simultaneous swing-frame orientation objectives. Two are
/// required by the canonical two-wheel support-loss transition; the previous
/// single slot made the physically valid zero-contact mode unrepresentable.
pub const FLOATING_ANGULAR_TASK_CAPACITY: usize = 2;
pub const FLOATING_TASK_DIAGNOSTIC_CAPACITY: usize =
    9 + FLOATING_ANGULAR_TASK_CAPACITY + FLOATING_POINT_TASK_CAPACITY;

/// Allocation-free discrete acceleration envelope for an authored joint.
///
/// Besides the next-tick velocity and position limits, this bounds the next
/// velocity by the speed that can still stop before either finite position
/// limit under `maximum_acceleration`. If the observed state is already
/// outside that braking envelope, the interval requests maximum inward
/// acceleration instead of manufacturing an invalid or inverted bound.
pub fn joint_acceleration_interval(
    position: f64,
    velocity: f64,
    lower_position: f64,
    upper_position: f64,
    maximum_velocity: f64,
    maximum_acceleration: f64,
    dt_seconds: f64,
) -> Option<(f64, f64)> {
    if !position.is_finite()
        || !velocity.is_finite()
        || lower_position.is_nan()
        || upper_position.is_nan()
        || lower_position > upper_position
        || !maximum_velocity.is_finite()
        || maximum_velocity <= 0.0
        || !maximum_acceleration.is_finite()
        || maximum_acceleration <= 0.0
        || !dt_seconds.is_finite()
        || dt_seconds <= 0.0
    {
        return None;
    }

    let dt_squared = dt_seconds * dt_seconds;
    let mut lower = (-maximum_acceleration).max((-maximum_velocity - velocity) / dt_seconds);
    let mut upper = maximum_acceleration.min((maximum_velocity - velocity) / dt_seconds);
    if lower_position.is_finite() {
        let distance = (position - lower_position).max(0.0);
        let safe_velocity = -(2.0 * maximum_acceleration * distance).sqrt();
        lower = lower
            .max((safe_velocity - velocity) / dt_seconds)
            .max(2.0 * (lower_position - position - velocity * dt_seconds) / dt_squared);
    }
    if upper_position.is_finite() {
        let distance = (upper_position - position).max(0.0);
        let safe_velocity = (2.0 * maximum_acceleration * distance).sqrt();
        upper = upper
            .min((safe_velocity - velocity) / dt_seconds)
            .min(2.0 * (upper_position - position - velocity * dt_seconds) / dt_squared);
    }

    // An already unsafe observation can require more than the declared
    // acceleration authority. Saturate toward the safe set; never invert the
    // box passed to the strict solver.
    lower = lower.clamp(-maximum_acceleration, maximum_acceleration);
    upper = upper.clamp(-maximum_acceleration, maximum_acceleration);
    if lower <= upper {
        Some((lower, upper))
    } else if velocity < 0.0 && lower_position.is_finite() {
        Some((maximum_acceleration, maximum_acceleration))
    } else if velocity > 0.0 && upper_position.is_finite() {
        Some((-maximum_acceleration, -maximum_acceleration))
    } else {
        None
    }
}

/// Intersection of the stopping-safe acceleration interval over the complete
/// axis-aligned observation-error box. Returning `None` means no single
/// acceleration is certified for every represented position/velocity corner;
/// callers must fail closed rather than silently use the nominal state.
#[allow(clippy::too_many_arguments)]
pub fn joint_acceleration_interval_with_observation_error(
    position: f64,
    velocity: f64,
    position_error: f64,
    velocity_error: f64,
    lower_position: f64,
    upper_position: f64,
    maximum_velocity: f64,
    maximum_acceleration: f64,
    dt_seconds: f64,
) -> Option<(f64, f64)> {
    if !position_error.is_finite()
        || position_error < 0.0
        || !velocity_error.is_finite()
        || velocity_error < 0.0
    {
        return None;
    }
    let mut robust_lower = -maximum_acceleration;
    let mut robust_upper = maximum_acceleration;
    for candidate_position in [position - position_error, position + position_error] {
        for candidate_velocity in [velocity - velocity_error, velocity + velocity_error] {
            let (lower, upper) = joint_acceleration_interval(
                candidate_position,
                candidate_velocity,
                lower_position,
                upper_position,
                maximum_velocity,
                maximum_acceleration,
                dt_seconds,
            )?;
            robust_lower = robust_lower.max(lower);
            robust_upper = robust_upper.min(upper);
        }
    }
    (robust_lower <= robust_upper).then_some((robust_lower, robust_upper))
}

/// Smooth, allocation-free braking command for a joint approaching its
/// authored velocity envelope. The command is exactly zero at the activation
/// threshold and reaches `-omega * velocity_limit` at the limit.
pub fn joint_velocity_envelope_acceleration(
    velocity: f64,
    maximum_velocity: f64,
    activation_fraction: f64,
    omega: f64,
    maximum_acceleration: f64,
) -> Option<f64> {
    if !velocity.is_finite()
        || !maximum_velocity.is_finite()
        || maximum_velocity <= 0.0
        || !activation_fraction.is_finite()
        || !(0.0..1.0).contains(&activation_fraction)
        || !omega.is_finite()
        || omega <= 0.0
        || !maximum_acceleration.is_finite()
        || maximum_acceleration <= 0.0
    {
        return None;
    }
    let utilization = velocity.abs() / maximum_velocity;
    if utilization <= activation_fraction {
        return Some(0.0);
    }
    let phase = ((utilization - activation_fraction) / (1.0 - activation_fraction)).clamp(0.0, 1.0);
    let smooth_phase = phase * phase * (3.0 - 2.0 * phase);
    Some(
        (-velocity.signum() * omega * maximum_velocity * smooth_phase)
            .clamp(-maximum_acceleration, maximum_acceleration),
    )
}

/// Smooth, allocation-free braking command when the current velocity can no
/// longer stop inside the remaining authored position headroom plus a bounded
/// reaction-time guard. This is a soft recovery request; the ordinary joint
/// acceleration interval remains the hard fail-closed authority boundary.
#[allow(clippy::too_many_arguments)]
pub fn joint_position_capture_acceleration(
    position: f64,
    velocity: f64,
    lower_position: f64,
    upper_position: f64,
    assumed_braking_acceleration: f64,
    reaction_time_seconds: f64,
    maximum_acceleration: f64,
) -> Option<f64> {
    if !position.is_finite()
        || !velocity.is_finite()
        || !lower_position.is_finite()
        || !upper_position.is_finite()
        || lower_position >= upper_position
        || !assumed_braking_acceleration.is_finite()
        || assumed_braking_acceleration <= 0.0
        || !reaction_time_seconds.is_finite()
        || reaction_time_seconds < 0.0
        || !maximum_acceleration.is_finite()
        || maximum_acceleration <= 0.0
    {
        return None;
    }
    if velocity == 0.0 {
        return Some(0.0);
    }
    let headroom = if velocity > 0.0 {
        upper_position - position
    } else {
        position - lower_position
    };
    let speed = velocity.abs();
    let required_headroom =
        speed * speed / (2.0 * assumed_braking_acceleration) + speed * reaction_time_seconds;
    if headroom >= required_headroom || required_headroom <= 0.0 {
        return Some(0.0);
    }
    let phase = ((required_headroom - headroom) / required_headroom).clamp(0.0, 1.0);
    let smooth_phase = phase * phase * (3.0 - 2.0 * phase);
    Some(-velocity.signum() * maximum_acceleration * smooth_phase)
}

/// Measured contact phase used to schedule soft authority without consulting
/// the authored motion phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MeasuredContactPhase {
    Unsupported = 0,
    SingleSupport = 1,
    Precontact = 2,
    MultiSupport = 3,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactPhaseAuthorityConfig {
    pub unsupported_joint_velocity_scale: f64,
    pub single_support_joint_velocity_scale: f64,
    pub precontact_joint_velocity_scale: f64,
    pub multi_support_joint_velocity_scale: f64,
}

impl Default for ContactPhaseAuthorityConfig {
    fn default() -> Self {
        Self {
            unsupported_joint_velocity_scale: 1.0,
            single_support_joint_velocity_scale: 1.0,
            precontact_joint_velocity_scale: 1.0,
            multi_support_joint_velocity_scale: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPhaseAuthorityOutput {
    pub phase: MeasuredContactPhase,
    pub joint_velocity_envelope_scale: f64,
}

/// Select soft-task authority from measured support state. This is deliberately
/// a pure scalar schedule: task rows, priority, and allocation remain fixed.
pub fn contact_phase_authority(
    established_support_count: usize,
    has_precontact_target: bool,
    config: ContactPhaseAuthorityConfig,
) -> Option<ContactPhaseAuthorityOutput> {
    let scales = [
        config.unsupported_joint_velocity_scale,
        config.single_support_joint_velocity_scale,
        config.precontact_joint_velocity_scale,
        config.multi_support_joint_velocity_scale,
    ];
    if scales
        .iter()
        .any(|scale| !scale.is_finite() || *scale < 0.0)
    {
        return None;
    }
    let phase = if established_support_count >= 2 {
        MeasuredContactPhase::MultiSupport
    } else if has_precontact_target {
        MeasuredContactPhase::Precontact
    } else if established_support_count == 1 {
        MeasuredContactPhase::SingleSupport
    } else {
        MeasuredContactPhase::Unsupported
    };
    let joint_velocity_envelope_scale = match phase {
        MeasuredContactPhase::Unsupported => config.unsupported_joint_velocity_scale,
        MeasuredContactPhase::SingleSupport => config.single_support_joint_velocity_scale,
        MeasuredContactPhase::Precontact => config.precontact_joint_velocity_scale,
        MeasuredContactPhase::MultiSupport => config.multi_support_joint_velocity_scale,
    };
    Some(ContactPhaseAuthorityOutput {
        phase,
        joint_velocity_envelope_scale,
    })
}

/// Slew a nonnegative authority scalar without overshoot. Keeping this in the
/// CPU core makes phase transitions deterministic and independent of adapter
/// wall-clock timing.
pub fn slew_contact_phase_authority(
    current_scale: f64,
    target_scale: f64,
    maximum_delta_per_tick: f64,
) -> Option<f64> {
    if !current_scale.is_finite()
        || current_scale < 0.0
        || !target_scale.is_finite()
        || target_scale < 0.0
        || !maximum_delta_per_tick.is_finite()
        || maximum_delta_per_tick <= 0.0
    {
        return None;
    }
    Some(
        current_scale
            + (target_scale - current_scale).clamp(-maximum_delta_per_tick, maximum_delta_per_tick),
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BalanceFeedbackAuthorityConfig {
    pub dcm_margin_full_scale_m: f64,
    pub dcm_margin_zero_scale_m: f64,
    pub attitude_full_scale_rad: f64,
    pub attitude_zero_scale_rad: f64,
    pub precontact_age_full_scale_ticks: usize,
    pub precontact_age_zero_scale_ticks: usize,
}

impl Default for BalanceFeedbackAuthorityConfig {
    fn default() -> Self {
        Self {
            dcm_margin_full_scale_m: 0.0,
            dcm_margin_zero_scale_m: 0.03,
            attitude_full_scale_rad: 5.0_f64.to_radians(),
            attitude_zero_scale_rad: 30.0_f64.to_radians(),
            precontact_age_full_scale_ticks: 150,
            precontact_age_zero_scale_ticks: 250,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BalanceFeedbackAuthorityOutput {
    pub joint_velocity_envelope_scale: f64,
    pub dcm_margin_scale: f64,
    pub attitude_scale: f64,
    pub precontact_age_scale: f64,
}

fn decreasing_smoothstep(value: f64, full_until: f64, zero_at: f64) -> f64 {
    let phase = ((value - full_until) / (zero_at - full_until)).clamp(0.0, 1.0);
    1.0 - phase * phase * (3.0 - 2.0 * phase)
}

/// Continuous, allocation-free authority target for a measured precontact
/// state. Capture need retains velocity-envelope authority, while growing root
/// attitude error and stale landing age release it back to the landing task.
pub fn balance_feedback_authority(
    dcm_support_margin_m: f64,
    root_attitude_error_rad: f64,
    precontact_age_ticks: usize,
    config: BalanceFeedbackAuthorityConfig,
) -> Option<BalanceFeedbackAuthorityOutput> {
    if !dcm_support_margin_m.is_finite()
        || !root_attitude_error_rad.is_finite()
        || root_attitude_error_rad < 0.0
        || !config.dcm_margin_full_scale_m.is_finite()
        || !config.dcm_margin_zero_scale_m.is_finite()
        || config.dcm_margin_full_scale_m >= config.dcm_margin_zero_scale_m
        || !config.attitude_full_scale_rad.is_finite()
        || !config.attitude_zero_scale_rad.is_finite()
        || config.attitude_full_scale_rad < 0.0
        || config.attitude_full_scale_rad >= config.attitude_zero_scale_rad
        || config.precontact_age_full_scale_ticks >= config.precontact_age_zero_scale_ticks
    {
        return None;
    }
    let dcm_margin_scale = decreasing_smoothstep(
        dcm_support_margin_m,
        config.dcm_margin_full_scale_m,
        config.dcm_margin_zero_scale_m,
    );
    let attitude_scale = decreasing_smoothstep(
        root_attitude_error_rad,
        config.attitude_full_scale_rad,
        config.attitude_zero_scale_rad,
    );
    let precontact_age_scale = decreasing_smoothstep(
        precontact_age_ticks as f64,
        config.precontact_age_full_scale_ticks as f64,
        config.precontact_age_zero_scale_ticks as f64,
    );
    Some(BalanceFeedbackAuthorityOutput {
        joint_velocity_envelope_scale: dcm_margin_scale * attitude_scale * precontact_age_scale,
        dcm_margin_scale,
        attitude_scale,
        precontact_age_scale,
    })
}

/// Configuration for the horizontal linear-inverted-pendulum balance law.
///
/// The natural frequency is derived from the measured CoM height above the
/// support plane. `feedback_gain_per_second` closes the DCM error dynamics;
/// it is deliberately independent of the WBC task weight and priority.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DcmBalanceConfig {
    pub gravity_mps2: f64,
    pub feedback_gain_per_second: f64,
    pub support_margin_m: f64,
    pub minimum_com_height_m: f64,
    pub maximum_com_height_m: f64,
    pub maximum_horizontal_acceleration_mps2: f64,
}

impl Default for DcmBalanceConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: 9.81,
            feedback_gain_per_second: 3.5,
            support_margin_m: 0.01,
            minimum_com_height_m: 0.2,
            maximum_com_height_m: 2.0,
            maximum_horizontal_acceleration_mps2: 25.0,
        }
    }
}

/// Allocation-free DCM/virtual-ZMP observation and command for one tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DcmBalanceOutput {
    pub desired_horizontal_acceleration_world: Vec3,
    pub dcm_world: Vec3,
    pub target_dcm_world: Vec3,
    pub virtual_zmp_world: Vec3,
    pub clipped_zmp_world: Vec3,
    pub natural_frequency_rad_s: f64,
    pub measured_com_height_m: f64,
    pub used_com_height_m: f64,
    pub com_height_was_clamped: bool,
    pub support_vertex_count: usize,
    /// Minimum signed inward half-space distance from measured DCM to the
    /// eroded support polygon. Positive is capturable inside; negative is out.
    pub dcm_support_margin_m: f64,
    pub zmp_was_clipped: bool,
}

fn cross2(lhs: Vector2<f64>, rhs: Vector2<f64>) -> f64 {
    lhs.x * rhs.y - lhs.y * rhs.x
}

/// Compute a dynamically consistent horizontal CoM acceleration from measured
/// CoM/DCM state and a desired DCM jet.
///
/// At most sixteen support points are accepted, matching the fixed four-target
/// by four-point floating WBC capacity. The points are reduced to a convex
/// hull in stack storage. The hull is eroded by `support_margin_m`, then the
/// virtual ZMP is projected to that polygon before applying
/// `c_ddot = omega^2 (c - zmp)`.
pub fn dcm_balance_acceleration(
    center_of_mass_world: Vec3,
    center_of_mass_velocity_world: Vec3,
    target_dcm_world: Vec3,
    target_dcm_velocity_world: Vec3,
    support_points_world: &[Vec3],
    config: DcmBalanceConfig,
) -> Option<DcmBalanceOutput> {
    const CAPACITY: usize = 16;
    const EPSILON: f64 = 1e-12;
    if support_points_world.len() < 3
        || support_points_world.len() > CAPACITY
        || !center_of_mass_world.iter().all(|value| value.is_finite())
        || !center_of_mass_velocity_world
            .iter()
            .all(|value| value.is_finite())
        || !target_dcm_world.iter().all(|value| value.is_finite())
        || !target_dcm_velocity_world
            .iter()
            .all(|value| value.is_finite())
        || !config.gravity_mps2.is_finite()
        || config.gravity_mps2 <= 0.0
        || !config.feedback_gain_per_second.is_finite()
        || config.feedback_gain_per_second <= 0.0
        || !config.support_margin_m.is_finite()
        || config.support_margin_m < 0.0
        || !config.minimum_com_height_m.is_finite()
        || !config.maximum_com_height_m.is_finite()
        || config.minimum_com_height_m <= 0.0
        || config.minimum_com_height_m > config.maximum_com_height_m
        || !config.maximum_horizontal_acceleration_mps2.is_finite()
        || config.maximum_horizontal_acceleration_mps2 <= 0.0
        || support_points_world
            .iter()
            .any(|point| !point.iter().all(|value| value.is_finite()))
    {
        return None;
    }

    let support_plane_z = support_points_world
        .iter()
        .map(|point| point.z)
        .sum::<f64>()
        / support_points_world.len() as f64;
    let measured_height = center_of_mass_world.z - support_plane_z;
    if !measured_height.is_finite() {
        return None;
    }
    let height = measured_height.clamp(config.minimum_com_height_m, config.maximum_com_height_m);
    let omega = (config.gravity_mps2 / height).sqrt();

    // Insertion sort is deterministic and faster than involving heap-backed
    // containers at this tiny fixed capacity.
    let mut sorted = [Vector2::zeros(); CAPACITY];
    let mut sorted_len = 0usize;
    for point in support_points_world {
        let point = Vector2::new(point.x, point.y);
        let mut slot = sorted_len;
        while slot > 0
            && (sorted[slot - 1].x > point.x
                || (sorted[slot - 1].x == point.x && sorted[slot - 1].y > point.y))
        {
            sorted[slot] = sorted[slot - 1];
            slot -= 1;
        }
        sorted[slot] = point;
        sorted_len += 1;
    }
    let mut unique_len = 0usize;
    for slot in 0..sorted_len {
        if unique_len == 0 || (sorted[slot] - sorted[unique_len - 1]).norm_squared() > EPSILON {
            sorted[unique_len] = sorted[slot];
            unique_len += 1;
        }
    }
    if unique_len < 3 {
        return None;
    }

    // Andrew's monotone chain. Collinear interior points are discarded, so
    // adjacent eroded-edge intersections remain well conditioned.
    let mut chain = [Vector2::zeros(); CAPACITY * 2];
    let mut chain_len = 0usize;
    for point in sorted.iter().take(unique_len).copied() {
        while chain_len >= 2
            && cross2(
                chain[chain_len - 1] - chain[chain_len - 2],
                point - chain[chain_len - 1],
            ) <= EPSILON
        {
            chain_len -= 1;
        }
        chain[chain_len] = point;
        chain_len += 1;
    }
    let upper_floor = chain_len + 1;
    for point in sorted.iter().take(unique_len - 1).rev().copied() {
        while chain_len >= upper_floor
            && cross2(
                chain[chain_len - 1] - chain[chain_len - 2],
                point - chain[chain_len - 1],
            ) <= EPSILON
        {
            chain_len -= 1;
        }
        chain[chain_len] = point;
        chain_len += 1;
    }
    // The final point repeats the first lower-hull vertex.
    chain_len -= 1;
    if chain_len < 3 {
        return None;
    }

    let hull_len = chain_len;
    let mut eroded = [Vector2::zeros(); CAPACITY * 2];
    for vertex in 0..hull_len {
        let previous = chain[(vertex + hull_len - 1) % hull_len];
        let current = chain[vertex];
        let next = chain[(vertex + 1) % hull_len];
        let previous_edge = current - previous;
        let current_edge = next - current;
        let previous_length = previous_edge.norm();
        let current_length = current_edge.norm();
        if previous_length <= EPSILON || current_length <= EPSILON {
            return None;
        }
        let previous_normal = Vector2::new(-previous_edge.y, previous_edge.x) / previous_length;
        let current_normal = Vector2::new(-current_edge.y, current_edge.x) / current_length;
        let previous_offset = previous_normal.dot(&current) + config.support_margin_m;
        let current_offset = current_normal.dot(&current) + config.support_margin_m;
        let determinant = cross2(previous_normal, current_normal);
        if determinant.abs() <= EPSILON {
            return None;
        }
        eroded[vertex] = Vector2::new(
            (previous_offset * current_normal.y - previous_normal.y * current_offset) / determinant,
            (previous_normal.x * current_offset - previous_offset * current_normal.x) / determinant,
        );
    }
    // A margin wider than the polygon can invert the offset intersections.
    // Reject it rather than silently expanding or changing the requested set.
    for point in eroded.iter().take(hull_len) {
        for edge in 0..hull_len {
            let start = chain[edge];
            let direction = chain[(edge + 1) % hull_len] - start;
            if cross2(direction, *point - start)
                < config.support_margin_m * direction.norm() - 1e-10
            {
                return None;
            }
        }
    }

    let measured_dcm = center_of_mass_world + center_of_mass_velocity_world / omega;
    let measured_dcm_horizontal = Vector2::new(measured_dcm.x, measured_dcm.y);
    let dcm_support_margin_m = (0..hull_len)
        .map(|edge| {
            let direction = eroded[(edge + 1) % hull_len] - eroded[edge];
            cross2(direction, measured_dcm_horizontal - eroded[edge]) / direction.norm()
        })
        .fold(f64::INFINITY, f64::min);
    let desired_dcm_velocity = target_dcm_velocity_world
        - config.feedback_gain_per_second * (measured_dcm - target_dcm_world);
    let mut virtual_zmp = measured_dcm - desired_dcm_velocity / omega;
    virtual_zmp.z = support_plane_z;
    let virtual_horizontal = Vector2::new(virtual_zmp.x, virtual_zmp.y);
    let mut clipped_horizontal = virtual_horizontal;
    let inside = (0..hull_len).all(|edge| {
        cross2(
            eroded[(edge + 1) % hull_len] - eroded[edge],
            virtual_horizontal - eroded[edge],
        ) >= -1e-10
    });
    if !inside {
        let mut best_distance_squared = f64::INFINITY;
        for edge in 0..hull_len {
            let start = eroded[edge];
            let segment = eroded[(edge + 1) % hull_len] - start;
            let parameter = ((virtual_horizontal - start).dot(&segment) / segment.norm_squared())
                .clamp(0.0, 1.0);
            let candidate = start + parameter * segment;
            let distance_squared = (candidate - virtual_horizontal).norm_squared();
            if distance_squared < best_distance_squared {
                best_distance_squared = distance_squared;
                clipped_horizontal = candidate;
            }
        }
    }
    let clipped_zmp = Vec3::new(clipped_horizontal.x, clipped_horizontal.y, support_plane_z);
    let mut desired_horizontal = Vec3::new(
        omega * omega * (center_of_mass_world.x - clipped_zmp.x),
        omega * omega * (center_of_mass_world.y - clipped_zmp.y),
        0.0,
    );
    let acceleration_norm = desired_horizontal.fixed_rows::<2>(0).norm();
    if acceleration_norm > config.maximum_horizontal_acceleration_mps2 {
        desired_horizontal *= config.maximum_horizontal_acceleration_mps2 / acceleration_norm;
    }

    Some(DcmBalanceOutput {
        desired_horizontal_acceleration_world: desired_horizontal,
        dcm_world: Vec3::new(measured_dcm.x, measured_dcm.y, support_plane_z),
        target_dcm_world: Vec3::new(target_dcm_world.x, target_dcm_world.y, support_plane_z),
        virtual_zmp_world: virtual_zmp,
        clipped_zmp_world: clipped_zmp,
        natural_frequency_rad_s: omega,
        measured_com_height_m: measured_height,
        used_com_height_m: height,
        com_height_was_clamped: measured_height != height,
        support_vertex_count: hull_len,
        dcm_support_margin_m,
        zmp_was_clipped: !inside,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ContactMode {
    /// No tangential or normal acceleration at the contact point.
    LockedPoint,
    /// Only normal acceleration is constrained. This is the touchdown
    /// transition mode before a point contact is safe to lock tangentially.
    NormalPoint,
    /// Tangent X is unconstrained (a skate/caster model); tangent Y and the
    /// normal are locked.
    RollingPoint,
    /// Nonholonomic wheel constraint at a wheel-center frame:
    /// `v_center_x + velocity_coefficient * qdot[coordinate] = 0`.
    /// Tangent Y and the normal remain locked.
    RollingWheel {
        coordinate: usize,
        velocity_coefficient: f64,
        /// Baumgarte gain applied to the no-slip velocity residual.
        velocity_stabilization_gain: f64,
        /// Absolute bound on the acceleration used to repair that residual.
        maximum_stabilization_acceleration: f64,
    },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ContactSpec {
    pub stable_id: u32,
    pub frame: FrameId,
    pub point_in_frame: Vec3,
    pub tangent_x_world: Vec3,
    pub tangent_y_world: Vec3,
    pub normal_world: Vec3,
    pub mode: ContactMode,
    /// Whether this force application point also emits kinematic contact
    /// rows. A finite support patch may need four independently bounded force
    /// points to represent its wrench cone, while only six independent rows
    /// are required to lock the rigid foot pose. Keeping these roles separate
    /// avoids an ill-conditioned redundant equality system.
    pub kinematic_enabled: bool,
    /// Commanded world acceleration at the contact point. This supports
    /// allocation-free Baumgarte stabilization without weakening contact rows.
    pub desired_point_acceleration_world: Vec3,
    pub friction_coefficient: f64,
    pub minimum_normal_force: f64,
    pub maximum_normal_force: f64,
    pub nominal_tangent_x_force: f64,
    pub nominal_tangent_y_force: f64,
    pub nominal_normal_force: f64,
}

/// A finite support patch over a contiguous range of contact-force slots.
///
/// The solver derives the convex hull from the current world positions of the
/// referenced force points. For every hull edge it emits the exact linear
/// center-of-pressure inequality over normal-force variables, eroded inward by
/// `minimum_margin_m`. A zero-load patch is unconstrained; any positively
/// loaded patch must keep its CoP inside the requested margin. An optional
/// aggregate normal-load row can require a minimum total support force while
/// preserving per-point load redistribution.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct SupportPatchSpec {
    pub stable_id: u32,
    pub first_contact: usize,
    pub contact_count: usize,
    pub minimum_margin_m: f64,
    /// Minimum aggregate normal load carried by this support patch. This is a
    /// patch-level row, allowing load to redistribute across force slots.
    pub minimum_total_normal_force: f64,
}

impl ContactSpec {
    pub fn horizontal(
        stable_id: u32,
        frame: FrameId,
        point_in_frame: Vec3,
        mode: ContactMode,
        friction_coefficient: f64,
        maximum_normal_force: f64,
        nominal_normal_force: f64,
    ) -> Self {
        Self {
            stable_id,
            frame,
            point_in_frame,
            tangent_x_world: Vec3::x(),
            tangent_y_world: Vec3::y(),
            normal_world: Vec3::z(),
            mode,
            kinematic_enabled: true,
            desired_point_acceleration_world: Vec3::zeros(),
            friction_coefficient,
            minimum_normal_force: 0.0,
            maximum_normal_force,
            nominal_tangent_x_force: 0.0,
            nominal_tangent_y_force: 0.0,
            nominal_normal_force,
        }
    }

    fn validate(self, model: &CompiledModel) -> bool {
        let basis = [
            self.tangent_x_world,
            self.tangent_y_world,
            self.normal_world,
        ];
        self.stable_id <= 0x03ff_ffff
            && self.frame.0 < model.bodies.len()
            && self.point_in_frame.iter().all(|value| value.is_finite())
            && basis
                .iter()
                .all(|axis| axis.iter().all(|value| value.is_finite()))
            && (self.tangent_x_world.norm() - 1.0).abs() <= 1e-8
            && (self.tangent_y_world.norm() - 1.0).abs() <= 1e-8
            && (self.normal_world.norm() - 1.0).abs() <= 1e-8
            && self.tangent_x_world.dot(&self.tangent_y_world).abs() <= 1e-8
            && self.tangent_x_world.dot(&self.normal_world).abs() <= 1e-8
            && self.tangent_y_world.dot(&self.normal_world).abs() <= 1e-8
            && self
                .tangent_x_world
                .cross(&self.tangent_y_world)
                .dot(&self.normal_world)
                >= 1.0 - 1e-8
            && self
                .desired_point_acceleration_world
                .iter()
                .all(|value| value.is_finite())
            && self.friction_coefficient.is_finite()
            && self.friction_coefficient >= 0.0
            && self.minimum_normal_force.is_finite()
            && self.minimum_normal_force >= 0.0
            && self.maximum_normal_force.is_finite()
            && self.maximum_normal_force >= self.minimum_normal_force
            && self.nominal_tangent_x_force.is_finite()
            && self.nominal_tangent_y_force.is_finite()
            && self.nominal_normal_force.is_finite()
            && self.nominal_normal_force >= self.minimum_normal_force
            && self.nominal_normal_force <= self.maximum_normal_force
            && match self.mode {
                ContactMode::LockedPoint | ContactMode::NormalPoint | ContactMode::RollingPoint => {
                    true
                }
                ContactMode::RollingWheel {
                    coordinate,
                    velocity_coefficient,
                    velocity_stabilization_gain,
                    maximum_stabilization_acceleration,
                } => {
                    coordinate < model.dof
                        && velocity_coefficient.is_finite()
                        && velocity_coefficient.abs() > 1e-9
                        && velocity_stabilization_gain.is_finite()
                        && velocity_stabilization_gain >= 0.0
                        && maximum_stabilization_acceleration.is_finite()
                        && maximum_stabilization_acceleration >= 0.0
                }
            }
    }
}

#[derive(Clone, Debug)]
pub struct DynamicWbcConfig {
    pub gravity_world: Vec3,
    pub acceleration_weight: f64,
    pub contact_force_weight: f64,
    pub actuator_torque_weight: f64,
    /// Hard-feasibility active-set budget. Full command solves use the default
    /// 64 iterations; planner-only query sessions may deliberately use less.
    pub maximum_feasibility_iterations: usize,
    /// Optional total Dykstra projection-sweep ceiling. `None` retains the
    /// exact row-count-scaled fallback; finite real-time profiles fail closed.
    pub maximum_feasibility_projection_sweeps: Option<usize>,
    /// Optional violation threshold for continuing a finite Dykstra prefix
    /// with the established row-scaled budget.
    pub feasibility_projection_continuation_violation_threshold: Option<f64>,
    /// Opt-in equality-first repair for the bounded active-set accelerator.
    pub repair_feasibility_equalities_before_inequalities: bool,
    /// Opt-in compact-span traversal for sparse Dykstra feasibility rows.
    pub use_feasibility_row_spans: bool,
    /// Opt-in workspace-local reuse of a terminal feasibility result for a
    /// bit-identical hard problem. Exact seeds still run every soft task;
    /// exhausted seeds remain fail-closed.
    pub reuse_identical_hard_feasibility_seed: bool,
    /// Opt-in continuation of an exhausted, bit-identical bounded problem in
    /// another ceiling-sized slice on the next call.
    pub continue_identical_exhausted_feasibility_prefix: bool,
    /// Optional locally linearized acceleration-level self-collision barrier
    /// for the floating solve. `None` preserves an explicit collision-disabled
    /// profile; trajectory command admission remains a separate gate.
    pub floating_collision_barrier: Option<CollisionAccelerationBarrierConfig>,
    /// Optional acceleration-level world-SDF barrier. A controller with this
    /// field enabled must be constructed with an explicit compiled world
    /// collision model; absence is rejected rather than silently treated free.
    pub floating_world_collision_barrier: Option<CollisionAccelerationBarrierConfig>,
}

impl Default for DynamicWbcConfig {
    fn default() -> Self {
        Self {
            gravity_world: Vec3::new(0.0, 0.0, -9.81),
            acceleration_weight: 1.0,
            contact_force_weight: 1.0,
            actuator_torque_weight: 1e-3,
            maximum_feasibility_iterations: 64,
            maximum_feasibility_projection_sweeps: None,
            feasibility_projection_continuation_violation_threshold: None,
            repair_feasibility_equalities_before_inequalities: false,
            use_feasibility_row_spans: false,
            reuse_identical_hard_feasibility_seed: false,
            continue_identical_exhausted_feasibility_prefix: false,
            floating_collision_barrier: None,
            floating_world_collision_barrier: None,
        }
    }
}

impl DynamicWbcConfig {
    fn validate(&self) -> bool {
        self.gravity_world
            .iter()
            .all(|component| component.is_finite())
            && [
                self.acceleration_weight,
                self.contact_force_weight,
                self.actuator_torque_weight,
            ]
            .into_iter()
            .all(|weight| weight.is_finite() && weight >= 0.0)
            && (1..=64).contains(&self.maximum_feasibility_iterations)
            && self
                .maximum_feasibility_projection_sweeps
                .is_none_or(|sweeps| sweeps > 0)
            && self
                .feasibility_projection_continuation_violation_threshold
                .is_none_or(|threshold| threshold.is_finite() && threshold >= 0.0)
            && self
                .floating_collision_barrier
                .is_none_or(CollisionAccelerationBarrierConfig::validate)
            && self
                .floating_world_collision_barrier
                .is_none_or(CollisionAccelerationBarrierConfig::validate)
    }
}

#[derive(Clone, Copy)]
pub struct DynamicWbcInput<'a> {
    pub state: &'a RobotState,
    pub desired_acceleration: &'a DVector<f64>,
    pub acceleration_bounds: &'a VelocityBounds,
    pub torque_bounds: &'a VelocityBounds,
    /// Optional exact actuator-space effort polytope. The solve variable is
    /// generalized effort, so each actuator row enforces
    /// `lower <= G[:, actuator]^T * tau_generalized <= upper`.
    pub actuator_effort: Option<ActuatorEffortInput<'a>>,
    pub contacts: &'a [ContactSpec],
}

#[derive(Clone, Copy)]
pub struct ActuatorEffortInput<'a> {
    pub actuation: &'a CompiledActuation,
    pub bounds: &'a VelocityBounds,
}

#[derive(Debug, Error)]
pub enum DynamicWbcError {
    #[error(transparent)]
    Model(#[from] ModelError),
    #[error(transparent)]
    Collision(#[from] CollisionError),
    #[error(transparent)]
    WorldCollision(#[from] WorldCollisionError),
    #[error("dynamic WBC configuration is invalid")]
    InvalidConfig,
    #[error("dynamic WBC input dimension or bound is invalid")]
    InvalidInput,
    #[error("contact {0} has an invalid frame, basis, friction, or force range")]
    InvalidContact(u32),
    #[error("contact count exceeds compiled capacity {capacity}: got {actual}")]
    ContactCapacity { capacity: usize, actual: usize },
    #[error("actuator count exceeds compiled capacity {capacity}: got {actual}")]
    ActuatorCapacity { capacity: usize, actual: usize },
    #[error(
        "current dynamic WBC requires a square invertible actuation map: {dof} generalized coordinates, {actuators} actuators"
    )]
    UnsupportedActuationTopology { dof: usize, actuators: usize },
    #[error("support patch {0} has an invalid contact range, basis, hull, or margin")]
    InvalidSupportPatch(u32),
    #[error("compiled dynamic-WBC constraint capacity was exceeded")]
    ConstraintCapacity,
}

#[derive(Debug)]
pub struct DynamicWbcOutput {
    pub status: SolveStatus,
    pub generalized_acceleration: DVector<f64>,
    /// Generalized joint effort solve variable. The legacy field name is kept
    /// for API compatibility; coupled actuator effort is `G^T * this vector`.
    pub actuator_torque: DVector<f64>,
    /// Columns are `[tangent_x, tangent_y, normal]` components per contact slot.
    pub contact_force_basis: DMatrix<f64>,
    /// World-expressed force per contact slot.
    pub contact_force_world: DMatrix<f64>,
    pub active_contacts: usize,
    pub dynamics_residual_linf: f64,
    pub contact_acceleration_residual_linf: f64,
    pub minimum_friction_margin: f64,
    pub minimum_torque_margin: f64,
    /// Minimum actuator-space effort headroom after the exact transmission
    /// map. Infinity means no actuator-space constraint was supplied.
    pub minimum_actuator_effort_margin: f64,
    pub limiting_actuator: Option<usize>,
    pub solve: SolveDiagnostics,
}

impl DynamicWbcOutput {
    pub fn workspace(dof: usize, maximum_contacts: usize, maximum_constraints: usize) -> Self {
        Self {
            status: SolveStatus::InvalidProblem,
            generalized_acceleration: DVector::zeros(dof),
            actuator_torque: DVector::zeros(dof),
            contact_force_basis: DMatrix::zeros(3, maximum_contacts),
            contact_force_world: DMatrix::zeros(3, maximum_contacts),
            active_contacts: 0,
            dynamics_residual_linf: f64::INFINITY,
            contact_acceleration_residual_linf: f64::INFINITY,
            minimum_friction_margin: f64::NEG_INFINITY,
            minimum_torque_margin: f64::NEG_INFINITY,
            minimum_actuator_effort_margin: f64::INFINITY,
            limiting_actuator: None,
            solve: SolveResult::workspace(0, maximum_constraints).diagnostics,
        }
    }

    pub fn generalized_effort(&self) -> &DVector<f64> {
        &self.actuator_torque
    }
}

#[derive(Clone, Copy)]
pub struct FloatingDynamicWbcInput<'a> {
    pub state: &'a RobotState,
    /// World-expressed root twist ordered `[angular; linear]`.
    pub root_twist_world: Motion6,
    /// Desired tangent acceleration ordered `[root angular; root linear; joints]`.
    pub desired_generalized_acceleration: &'a DVector<f64>,
    /// Lexicographic placement of each semantic acceleration block.
    pub task_priorities: FloatingTaskPriorities,
    /// Multipliers relative to the configured acceleration-task weight.
    pub task_weights: FloatingTaskWeights,
    /// Multiplier for the joint-posture block; zero disables it.
    pub joint_posture_weight: f64,
    /// Optional fixed-subset joint acceleration task, used for actuators such
    /// as balance wheels that must sit above the general posture objective.
    pub joint_acceleration_task: Option<FloatingJointAccelerationTask<'a>>,
    pub center_of_mass_task: Option<FloatingCenterOfMassTask>,
    pub centroidal_angular_momentum_task: Option<FloatingCentroidalAngularMomentumTask>,
    /// Fixed-capacity angular-acceleration tasks for swing-foot orientation
    /// and other floating frames. Stable IDs must be strictly increasing.
    pub frame_angular_acceleration_tasks: &'a [FloatingFrameAngularAccelerationTask],
    /// Fixed-capacity Cartesian acceleration tasks for swing feet, hands, and
    /// other floating-body points. Stable IDs must be strictly increasing.
    pub point_acceleration_tasks: &'a [FloatingPointAccelerationTask],
    pub generalized_acceleration_bounds: &'a VelocityBounds,
    pub torque_bounds: &'a VelocityBounds,
    /// Optional exact actuator-space effort polytope; see
    /// [`ActuatorEffortInput`].
    pub actuator_effort: Option<ActuatorEffortInput<'a>>,
    pub contacts: &'a [ContactSpec],
    /// Finite contact patches with hard center-of-pressure margins. Ranges
    /// must be disjoint, ordered, and refer to contacts in this input.
    pub support_patches: &'a [SupportPatchSpec],
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingJointAccelerationTask<'a> {
    /// Strictly increasing generalized joint-coordinate indexes.
    pub coordinates: &'a [usize],
    pub desired_accelerations: &'a [f64],
    pub priority: Priority,
    /// Multiplier relative to the configured acceleration-task weight.
    pub weight: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingCenterOfMassTask {
    pub desired_acceleration_world: Vec3,
    /// Use only world X/Y rows. Vertical support is then owned by the root
    /// height task instead of redundantly constraining CoM height.
    pub horizontal_only: bool,
    pub priority: Priority,
    /// Multiplier relative to the configured acceleration-task weight.
    pub weight: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingCentroidalAngularMomentumTask {
    /// Desired angular-momentum rate about the system CoM in control world.
    pub desired_rate_world: Vec3,
    pub priority: Priority,
    /// Multiplier relative to the configured acceleration-task weight.
    pub weight: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingPointAccelerationTask {
    pub stable_id: u32,
    pub frame: FrameId,
    pub point_in_frame: Vec3,
    pub desired_acceleration_world: Vec3,
    pub priority: Priority,
    /// Multiplier relative to the configured acceleration-task weight.
    pub weight: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingFrameAngularAccelerationTask {
    pub stable_id: u32,
    pub frame: FrameId,
    pub desired_angular_acceleration_world: Vec3,
    pub priority: Priority,
    /// Multiplier relative to the configured acceleration-task weight.
    pub weight: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingTaskPriorities {
    pub root_angular: Priority,
    pub root_horizontal: Priority,
    pub root_height: Priority,
    pub joint_posture: Priority,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingTaskWeights {
    pub root_angular: f64,
    pub root_horizontal: f64,
    pub root_height: f64,
    pub joint_posture: f64,
}

impl Default for FloatingTaskWeights {
    fn default() -> Self {
        Self {
            root_angular: 1.0,
            root_horizontal: 1.0,
            root_height: 1.0,
            joint_posture: 1.0,
        }
    }
}

impl Default for FloatingTaskPriorities {
    fn default() -> Self {
        Self {
            root_angular: Priority::Intent,
            root_horizontal: Priority::Intent,
            root_height: Priority::Intent,
            joint_posture: Priority::Intent,
        }
    }
}

/// Allocation-free, stable provenance for one task in the floating-base solve.
///
/// Residuals are reported in the task's physical units before applying its
/// scalar weight. `clipped` means the task's priority level required slack; it
/// does not mean the hard dynamics or contact constraints were infeasible.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct FloatingTaskResidual {
    pub stable_id: u32,
    pub kind: TaskKind,
    pub priority: Priority,
    pub active: bool,
    pub clipped: bool,
    pub rows: usize,
    pub l2: f64,
    pub rms: f64,
}

impl FloatingTaskResidual {
    pub const fn inactive() -> Self {
        Self {
            stable_id: 0,
            kind: TaskKind::Acceleration,
            priority: Priority::Style,
            active: false,
            clipped: false,
            rows: 0,
            l2: 0.0,
            rms: 0.0,
        }
    }
}

impl Default for FloatingTaskResidual {
    fn default() -> Self {
        Self::inactive()
    }
}

#[derive(Debug)]
pub struct FloatingDynamicWbcOutput {
    pub status: SolveStatus,
    /// Tangent acceleration ordered `[root angular; root linear; joints]`.
    pub generalized_acceleration: DVector<f64>,
    /// Generalized joint effort solve variable. The legacy field name is kept
    /// for API compatibility; coupled actuator effort is `G^T * this vector`.
    pub actuator_torque: DVector<f64>,
    /// Columns are `[tangent_x, tangent_y, normal]` components per contact slot.
    pub contact_force_basis: DMatrix<f64>,
    /// World-expressed force per contact slot.
    pub contact_force_world: DMatrix<f64>,
    pub active_contacts: usize,
    pub dynamics_residual_linf: f64,
    pub contact_acceleration_residual_linf: f64,
    pub minimum_friction_margin: f64,
    /// Minimum geometric CoP distance to any loaded support-patch boundary.
    /// Positive is inside. Infinity means no declared patch carried load.
    pub minimum_support_margin_m: f64,
    /// Stable ID of the loaded patch that owns `minimum_support_margin_m`.
    /// `None` means that no declared patch carried appreciable normal load.
    pub limiting_support_patch: Option<u32>,
    pub minimum_torque_margin: f64,
    /// Minimum actuator-space effort headroom after the exact transmission
    /// map. Infinity means no actuator-space constraint was supplied.
    pub minimum_actuator_effort_margin: f64,
    pub limiting_actuator: Option<usize>,
    /// Closest represented pair and limiting acceleration-level viability row.
    /// This evidence is independent of support, effort, solver, and trajectory
    /// command-admission diagnostics.
    pub collision_barrier: CollisionAccelerationBarrierEvidence,
    /// Static-world SDF viability evidence. This remains independently typed
    /// from self-collision and commanded-segment admission; the local row does
    /// not imply swept-world clearance or realized plant response.
    pub world_collision_barrier: WorldCollisionBarrierEvidence,
    /// Fixed task slots preserve stable IDs without allocating in the hot loop.
    pub task_residuals: [FloatingTaskResidual; FLOATING_TASK_DIAGNOSTIC_CAPACITY],
    pub solve: SolveDiagnostics,
}

impl FloatingDynamicWbcOutput {
    pub fn workspace(dof: usize, maximum_contacts: usize, maximum_constraints: usize) -> Self {
        Self {
            status: SolveStatus::InvalidProblem,
            generalized_acceleration: DVector::zeros(dof + 6),
            actuator_torque: DVector::zeros(dof),
            contact_force_basis: DMatrix::zeros(3, maximum_contacts),
            contact_force_world: DMatrix::zeros(3, maximum_contacts),
            active_contacts: 0,
            dynamics_residual_linf: f64::INFINITY,
            contact_acceleration_residual_linf: f64::INFINITY,
            minimum_friction_margin: f64::NEG_INFINITY,
            minimum_support_margin_m: f64::NEG_INFINITY,
            limiting_support_patch: None,
            minimum_torque_margin: f64::NEG_INFINITY,
            minimum_actuator_effort_margin: f64::INFINITY,
            limiting_actuator: None,
            collision_barrier: CollisionAccelerationBarrierEvidence::disabled(),
            world_collision_barrier: WorldCollisionBarrierEvidence::disabled(),
            task_residuals: [FloatingTaskResidual::inactive(); FLOATING_TASK_DIAGNOSTIC_CAPACITY],
            solve: SolveResult::workspace(0, maximum_constraints).diagnostics,
        }
    }

    pub fn generalized_effort(&self) -> &DVector<f64> {
        &self.actuator_torque
    }

    /// Copy a solved query into an existing caller-owned workspace without
    /// changing its fixed layout. Returns false on a dimension mismatch.
    pub fn copy_from_same_layout(&mut self, source: &Self) -> bool {
        if self.generalized_acceleration.len() != source.generalized_acceleration.len()
            || self.actuator_torque.len() != source.actuator_torque.len()
            || self.contact_force_basis.shape() != source.contact_force_basis.shape()
            || self.contact_force_world.shape() != source.contact_force_world.shape()
        {
            return false;
        }
        self.status = source.status;
        self.generalized_acceleration
            .copy_from(&source.generalized_acceleration);
        self.actuator_torque.copy_from(&source.actuator_torque);
        self.contact_force_basis
            .copy_from(&source.contact_force_basis);
        self.contact_force_world
            .copy_from(&source.contact_force_world);
        self.active_contacts = source.active_contacts;
        self.dynamics_residual_linf = source.dynamics_residual_linf;
        self.contact_acceleration_residual_linf = source.contact_acceleration_residual_linf;
        self.minimum_friction_margin = source.minimum_friction_margin;
        self.minimum_support_margin_m = source.minimum_support_margin_m;
        self.limiting_support_patch = source.limiting_support_patch;
        self.minimum_torque_margin = source.minimum_torque_margin;
        self.minimum_actuator_effort_margin = source.minimum_actuator_effort_margin;
        self.limiting_actuator = source.limiting_actuator;
        self.collision_barrier = source.collision_barrier;
        self.world_collision_barrier = source.world_collision_barrier;
        self.task_residuals = source.task_residuals;
        self.solve.clone_from(&source.solve);
        true
    }
}

#[derive(Clone, Debug)]
pub struct DynamicWbcScratch {
    pub model: ModelCache,
    pub dynamics: DynamicsCache,
    mass_matrix: DMatrix<f64>,
    bias: DVector<f64>,
    contact_jacobian: DMatrix<f64>,
    tasks: Vec<Task>,
    constraints: ConstraintBuffer,
    bounds: VelocityBounds,
    solver_workspace: SolverWorkspace,
    solve_result: SolveResult,
    maximum_contacts: usize,
    maximum_actuators: usize,
}

impl DynamicWbcScratch {
    pub fn new(model: &CompiledModel, maximum_contacts: usize) -> Self {
        Self::new_with_actuator_capacity(model, maximum_contacts, model.dof)
    }

    pub fn new_with_actuator_capacity(
        model: &CompiledModel,
        maximum_contacts: usize,
        maximum_actuators: usize,
    ) -> Self {
        let dof = model.dof;
        let variables = dof
            .saturating_mul(2)
            .saturating_add(maximum_contacts.saturating_mul(3));
        let maximum_constraints = dof
            .saturating_add(maximum_contacts.saturating_mul(7))
            .saturating_add(maximum_actuators);
        let semantic_task_rows = dof
            .saturating_mul(2)
            .saturating_add(maximum_contacts.saturating_mul(3));
        let maximum_feasibility_rows = variables
            .saturating_mul(2)
            .saturating_add(maximum_constraints.saturating_mul(3));
        let maximum_task_rows = semantic_task_rows.max(maximum_feasibility_rows);
        let tasks = vec![
            dynamic_task(dof, variables, 1, TaskKind::Acceleration, Priority::Intent),
            dynamic_task(
                maximum_contacts.saturating_mul(3),
                variables,
                2,
                TaskKind::ContactWrench,
                Priority::Preference,
            ),
            dynamic_task(
                dof,
                variables,
                3,
                TaskKind::ActuatorTorque,
                Priority::Preference,
            ),
        ];
        Self {
            model: ModelCache::new(model),
            dynamics: DynamicsCache::new(model),
            mass_matrix: DMatrix::zeros(dof, dof),
            bias: DVector::zeros(dof),
            contact_jacobian: DMatrix::zeros(3, dof),
            tasks,
            constraints: ConstraintBuffer::new(variables, maximum_constraints),
            bounds: VelocityBounds::unbounded(variables),
            solver_workspace: SolverWorkspace::new(
                variables,
                maximum_task_rows,
                3,
                maximum_constraints,
            ),
            solve_result: SolveResult::workspace(variables, maximum_constraints),
            maximum_contacts,
            maximum_actuators,
        }
    }

    pub fn maximum_contacts(&self) -> usize {
        self.maximum_contacts
    }

    pub fn maximum_actuators(&self) -> usize {
        self.maximum_actuators
    }
}

#[derive(Clone, Debug)]
pub struct FloatingDynamicWbcScratch {
    pub model: ModelCache,
    pub dynamics: DynamicsCache,
    mass_matrix: DMatrix<f64>,
    bias: DVector<f64>,
    contact_jacobian: DMatrix<f64>,
    point_task_jacobian: DMatrix<f64>,
    angular_task_jacobian: DMatrix<f64>,
    center_of_mass_jacobian: DMatrix<f64>,
    center_of_mass_bias_acceleration_world: Vec3,
    collision_barrier: CollisionAccelerationBarrierScratch,
    world_collision_barrier: WorldCollisionBarrierScratch,
    tasks: Vec<Task>,
    constraints: ConstraintBuffer,
    bounds: VelocityBounds,
    solver_workspace: SolverWorkspace,
    solve_result: SolveResult,
    maximum_contacts: usize,
    maximum_actuators: usize,
}

impl FloatingDynamicWbcScratch {
    pub fn new(model: &CompiledModel, maximum_contacts: usize) -> Self {
        Self::new_with_actuator_capacity(model, maximum_contacts, model.dof)
    }

    pub fn new_with_actuator_capacity(
        model: &CompiledModel,
        maximum_contacts: usize,
        maximum_actuators: usize,
    ) -> Self {
        let self_collision_capacity = CompiledCollisionModel::compile(model)
            .validation_pairs
            .len();
        Self::new_with_layout(
            model,
            maximum_contacts,
            maximum_actuators,
            true,
            self_collision_capacity,
            0,
        )
    }

    /// Preallocate the compact `[generalized acceleration, contact force]`
    /// layout used when generalized effort is supplied as a dynamics input.
    pub fn new_fixed_effort(model: &CompiledModel, maximum_contacts: usize) -> Self {
        let self_collision_capacity = CompiledCollisionModel::compile(model)
            .validation_pairs
            .len();
        Self::new_with_layout(
            model,
            maximum_contacts,
            0,
            false,
            self_collision_capacity,
            0,
        )
    }

    /// Construct scratch for a controller whose compiled collision layouts
    /// are already known. Runtime solve methods never grow these capacities.
    pub fn new_with_collision_capacities(
        model: &CompiledModel,
        maximum_contacts: usize,
        maximum_actuators: usize,
        self_collision_capacity: usize,
        world_collision_capacity: usize,
    ) -> Self {
        Self::new_with_layout(
            model,
            maximum_contacts,
            maximum_actuators,
            true,
            self_collision_capacity,
            world_collision_capacity,
        )
    }

    pub fn new_fixed_effort_with_collision_capacities(
        model: &CompiledModel,
        maximum_contacts: usize,
        self_collision_capacity: usize,
        world_collision_capacity: usize,
    ) -> Self {
        Self::new_with_layout(
            model,
            maximum_contacts,
            0,
            false,
            self_collision_capacity,
            world_collision_capacity,
        )
    }

    fn new_with_layout(
        model: &CompiledModel,
        maximum_contacts: usize,
        maximum_actuators: usize,
        include_effort_variables: bool,
        collision_pair_capacity: usize,
        world_collision_probe_capacity: usize,
    ) -> Self {
        let dof = model.dof;
        let generalized_dof = dof.saturating_add(6);
        let variables = generalized_dof
            .saturating_add(if include_effort_variables { dof } else { 0 })
            .saturating_add(maximum_contacts.saturating_mul(3));
        let maximum_constraints = generalized_dof
            .saturating_add(maximum_contacts.saturating_mul(8))
            .saturating_add(maximum_actuators)
            .saturating_add(collision_pair_capacity)
            .saturating_add(world_collision_probe_capacity);
        let semantic_task_rows = generalized_dof
            .saturating_add(dof.saturating_mul(2))
            .saturating_add(3)
            .saturating_add(maximum_contacts.saturating_mul(3))
            .saturating_add(FLOATING_ANGULAR_TASK_CAPACITY.saturating_mul(3))
            .saturating_add(FLOATING_POINT_TASK_CAPACITY.saturating_mul(3));
        let maximum_feasibility_rows = variables
            .saturating_mul(2)
            .saturating_add(maximum_constraints.saturating_mul(3));
        let maximum_task_rows = semantic_task_rows.max(maximum_feasibility_rows);
        let mut tasks = vec![
            dynamic_task(3, variables, 1, TaskKind::Orientation, Priority::Viability),
            dynamic_task(2, variables, 2, TaskKind::Acceleration, Priority::Viability),
            dynamic_task(1, variables, 3, TaskKind::Acceleration, Priority::Intent),
            dynamic_task(dof, variables, 4, TaskKind::Posture, Priority::Preference),
            dynamic_task(
                dof,
                variables,
                5,
                TaskKind::Acceleration,
                Priority::Viability,
            ),
            dynamic_task(3, variables, 6, TaskKind::CenterOfMass, Priority::Intent),
            dynamic_task(
                3,
                variables,
                7,
                TaskKind::CentroidalMomentum,
                Priority::Viability,
            ),
            dynamic_task(
                maximum_contacts.saturating_mul(3),
                variables,
                8,
                TaskKind::ContactWrench,
                Priority::Style,
            ),
            dynamic_task(dof, variables, 9, TaskKind::ActuatorTorque, Priority::Style),
        ];
        for slot in 0..FLOATING_ANGULAR_TASK_CAPACITY {
            tasks.push(dynamic_task(
                3,
                variables,
                10 + slot as u32,
                TaskKind::Orientation,
                Priority::Intent,
            ));
        }
        for slot in 0..FLOATING_POINT_TASK_CAPACITY {
            tasks.push(dynamic_task(
                3,
                variables,
                10 + FLOATING_ANGULAR_TASK_CAPACITY as u32 + slot as u32,
                TaskKind::Point,
                Priority::Intent,
            ));
        }
        Self {
            model: ModelCache::new(model),
            dynamics: DynamicsCache::new(model),
            mass_matrix: DMatrix::zeros(generalized_dof, generalized_dof),
            bias: DVector::zeros(generalized_dof),
            contact_jacobian: DMatrix::zeros(3, generalized_dof),
            point_task_jacobian: DMatrix::zeros(3, generalized_dof),
            angular_task_jacobian: DMatrix::zeros(3, generalized_dof),
            center_of_mass_jacobian: DMatrix::zeros(3, generalized_dof),
            center_of_mass_bias_acceleration_world: Vec3::zeros(),
            collision_barrier: CollisionAccelerationBarrierScratch::new(
                generalized_dof,
                collision_pair_capacity,
            ),
            world_collision_barrier: WorldCollisionBarrierScratch::new(
                generalized_dof,
                world_collision_probe_capacity,
            ),
            tasks,
            constraints: ConstraintBuffer::new(variables, maximum_constraints),
            bounds: VelocityBounds::unbounded(variables),
            solver_workspace: SolverWorkspace::new(
                variables,
                maximum_task_rows,
                FLOATING_TASK_DIAGNOSTIC_CAPACITY,
                maximum_constraints,
            ),
            solve_result: SolveResult::workspace(variables, maximum_constraints),
            maximum_contacts,
            maximum_actuators,
        }
    }

    pub fn maximum_contacts(&self) -> usize {
        self.maximum_contacts
    }

    pub fn maximum_actuators(&self) -> usize {
        self.maximum_actuators
    }

    /// Import a copied hard-feasibility witness from an independent WBC
    /// scratch arena. The solver revalidates the complete hard-problem key on
    /// its next call and never shares mutable matrices or equality factors.
    pub fn import_hard_feasibility_witness_from(&mut self, source: &Self) -> bool {
        self.solver_workspace
            .import_hard_feasibility_witness_from(&source.solver_workspace)
    }
}

pub struct DynamicWbc {
    model: CompiledModel,
    config: DynamicWbcConfig,
    solver: HierarchicalSolver,
}

pub struct FloatingDynamicWbc {
    model: CompiledModel,
    config: DynamicWbcConfig,
    collision: Option<CompiledCollisionModel>,
    world_collision: Option<CompiledWorldCollisionModel>,
    solver: HierarchicalSolver,
}

impl DynamicWbc {
    pub fn new(model: CompiledModel, config: DynamicWbcConfig) -> Result<Self, DynamicWbcError> {
        if !config.validate() {
            return Err(DynamicWbcError::InvalidConfig);
        }
        Ok(Self {
            model,
            config,
            solver: HierarchicalSolver::default(),
        })
    }

    pub fn model(&self) -> &CompiledModel {
        &self.model
    }

    pub fn solve(
        &self,
        input: DynamicWbcInput<'_>,
        maximum_contacts: usize,
    ) -> Result<DynamicWbcOutput, DynamicWbcError> {
        let actuator_count = input
            .actuator_effort
            .map_or(0, |spec| spec.actuation.actuators.len());
        let maximum_constraints = self
            .model
            .dof
            .saturating_add(maximum_contacts.saturating_mul(7))
            .saturating_add(actuator_count);
        let mut scratch = DynamicWbcScratch::new_with_actuator_capacity(
            &self.model,
            maximum_contacts,
            actuator_count,
        );
        let mut output =
            DynamicWbcOutput::workspace(self.model.dof, maximum_contacts, maximum_constraints);
        self.solve_into(input, &mut output, &mut scratch)?;
        Ok(output)
    }

    pub fn solve_into(
        &self,
        input: DynamicWbcInput<'_>,
        output: &mut DynamicWbcOutput,
        scratch: &mut DynamicWbcScratch,
    ) -> Result<(), DynamicWbcError> {
        let dof = self.model.dof;
        let contacts = input.contacts.len();
        if contacts > scratch.maximum_contacts {
            return Err(DynamicWbcError::ContactCapacity {
                capacity: scratch.maximum_contacts,
                actual: contacts,
            });
        }
        validate_actuator_effort_input(
            input.actuator_effort,
            &self.model,
            scratch.maximum_actuators,
        )?;
        input.state.validate(&self.model)?;
        if input.desired_acceleration.len() != dof
            || !input
                .desired_acceleration
                .iter()
                .all(|value| value.is_finite())
            || !input.acceleration_bounds.validate(dof)
            || !input.torque_bounds.validate(dof)
            || output.generalized_acceleration.len() != dof
            || output.actuator_torque.len() != dof
            || output.contact_force_basis.ncols() != scratch.maximum_contacts
            || output.contact_force_world.ncols() != scratch.maximum_contacts
        {
            return Err(DynamicWbcError::InvalidInput);
        }
        for contact in input.contacts {
            if !contact.validate(&self.model) {
                return Err(DynamicWbcError::InvalidContact(contact.stable_id));
            }
        }
        if input
            .contacts
            .windows(2)
            .any(|pair| pair[0].stable_id >= pair[1].stable_id)
        {
            return Err(DynamicWbcError::InvalidInput);
        }
        self.model
            .forward_kinematics(input.state, &mut scratch.model)?;
        self.model.mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_matrix,
        )?;
        self.model.bias_forces_into(
            input.state,
            self.config.gravity_world,
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.bias,
        )?;

        let variables = dof * 2 + scratch.maximum_contacts * 3;
        prepare_tasks(
            &mut scratch.tasks,
            dof,
            scratch.maximum_contacts,
            input,
            &self.config,
        );
        prepare_bounds(&mut scratch.bounds, dof, scratch.maximum_contacts, input);
        scratch.constraints.begin();
        emit_dynamics_rows(
            &scratch.mass_matrix,
            &scratch.bias,
            dof,
            &mut scratch.constraints,
        )?;
        emit_actuator_effort_rows(&mut scratch.constraints, input.actuator_effort, dof)?;

        for (contact_index, contact) in input.contacts.iter().copied().enumerate() {
            self.model.point_jacobian_into(
                &scratch.model,
                contact.frame,
                contact.point_in_frame,
                &mut scratch.contact_jacobian,
            )?;
            let bias_acceleration = self.model.point_bias_acceleration_world(
                contact.frame,
                contact.point_in_frame,
                &scratch.model,
                &scratch.dynamics,
            )?;
            augment_dynamics_with_contact(
                &mut scratch.constraints,
                &scratch.contact_jacobian,
                contact,
                dof,
                contact_index,
            );
            emit_contact_acceleration_rows(
                &mut scratch.constraints,
                &scratch.contact_jacobian,
                bias_acceleration,
                contact,
                dof,
                input.state.v.as_slice(),
            )?;
            emit_friction_rows(&mut scratch.constraints, contact, dof, contact_index)?;
        }

        debug_assert_eq!(scratch.bounds.lower.len(), variables);
        self.solver.solve_constrained_into(
            variables,
            &scratch.tasks,
            &scratch.bounds,
            scratch.constraints.active(),
            &mut scratch.solve_result,
            &mut scratch.solver_workspace,
        );
        fill_output(
            output,
            &scratch.solve_result,
            &scratch.constraints,
            input,
            dof,
            scratch.maximum_contacts,
        );
        Ok(())
    }
}

impl FloatingDynamicWbc {
    pub fn new(model: CompiledModel, config: DynamicWbcConfig) -> Result<Self, DynamicWbcError> {
        if config.floating_world_collision_barrier.is_some() {
            return Err(DynamicWbcError::InvalidConfig);
        }
        Self::new_impl(model, config, None)
    }

    pub fn new_with_world_collision(
        model: CompiledModel,
        config: DynamicWbcConfig,
        world_collision: CompiledWorldCollisionModel,
    ) -> Result<Self, DynamicWbcError> {
        if config.floating_world_collision_barrier.is_none() {
            return Err(DynamicWbcError::InvalidConfig);
        }
        Self::new_impl(model, config, Some(world_collision))
    }

    fn new_impl(
        model: CompiledModel,
        config: DynamicWbcConfig,
        world_collision: Option<CompiledWorldCollisionModel>,
    ) -> Result<Self, DynamicWbcError> {
        if !config.validate() {
            return Err(DynamicWbcError::InvalidConfig);
        }
        if world_collision
            .as_ref()
            .is_some_and(|collision| collision.field.validate().is_err())
        {
            return Err(DynamicWbcError::InvalidConfig);
        }
        let collision = config
            .floating_collision_barrier
            .map(|_| CompiledCollisionModel::compile(&model));
        let maximum_feasibility_iterations = config.maximum_feasibility_iterations;
        let maximum_feasibility_projection_sweeps = config.maximum_feasibility_projection_sweeps;
        let feasibility_projection_continuation_violation_threshold =
            config.feasibility_projection_continuation_violation_threshold;
        let repair_feasibility_equalities_before_inequalities =
            config.repair_feasibility_equalities_before_inequalities;
        let use_feasibility_row_spans = config.use_feasibility_row_spans;
        let reuse_identical_hard_feasibility_seed = config.reuse_identical_hard_feasibility_seed;
        let continue_identical_exhausted_feasibility_prefix =
            config.continue_identical_exhausted_feasibility_prefix;
        Ok(Self {
            model,
            config,
            collision,
            world_collision,
            solver: HierarchicalSolver {
                singular_value_tolerance: 1e-12,
                task_singular_value_damping: 1e-8,
                maximum_feasibility_iterations,
                maximum_feasibility_projection_sweeps,
                feasibility_projection_continuation_violation_threshold,
                repair_feasibility_equalities_before_inequalities,
                use_feasibility_row_spans,
                reuse_identical_hard_feasibility_seed,
                continue_identical_exhausted_feasibility_prefix,
            },
        })
    }

    pub fn model(&self) -> &CompiledModel {
        &self.model
    }

    pub fn collision_pair_count(&self) -> usize {
        self.collision
            .as_ref()
            .map_or(0, |collision| collision.validation_pairs.len())
    }

    pub fn world_collision_probe_count(&self) -> usize {
        self.world_collision
            .as_ref()
            .map_or(0, |collision| collision.probes.len())
    }

    pub fn world_collision_probe_body(&self, stable_id: u32) -> Option<crate::model::BodyId> {
        self.world_collision
            .as_ref()
            .and_then(|collision| collision.probe_body(stable_id))
    }

    pub fn maximum_constraint_count(
        &self,
        maximum_contacts: usize,
        maximum_actuators: usize,
    ) -> usize {
        self.model
            .dof
            .saturating_add(6)
            .saturating_add(maximum_contacts.saturating_mul(8))
            .saturating_add(maximum_actuators)
            .saturating_add(self.collision_pair_count())
            .saturating_add(self.world_collision_probe_count())
    }

    pub fn scratch(
        &self,
        maximum_contacts: usize,
        maximum_actuators: usize,
    ) -> FloatingDynamicWbcScratch {
        FloatingDynamicWbcScratch::new_with_collision_capacities(
            &self.model,
            maximum_contacts,
            maximum_actuators,
            self.collision_pair_count(),
            self.world_collision_probe_count(),
        )
    }

    pub fn fixed_effort_scratch(&self, maximum_contacts: usize) -> FloatingDynamicWbcScratch {
        FloatingDynamicWbcScratch::new_fixed_effort_with_collision_capacities(
            &self.model,
            maximum_contacts,
            self.collision_pair_count(),
            self.world_collision_probe_count(),
        )
    }

    pub fn solve(
        &self,
        input: FloatingDynamicWbcInput<'_>,
        maximum_contacts: usize,
    ) -> Result<FloatingDynamicWbcOutput, DynamicWbcError> {
        let actuator_count = input
            .actuator_effort
            .map_or(0, |spec| spec.actuation.actuators.len());
        let maximum_constraints = self.maximum_constraint_count(maximum_contacts, actuator_count);
        let mut scratch = self.scratch(maximum_contacts, actuator_count);
        let mut output = FloatingDynamicWbcOutput::workspace(
            self.model.dof,
            maximum_contacts,
            maximum_constraints,
        );
        self.solve_into(input, &mut output, &mut scratch)?;
        Ok(output)
    }

    pub fn solve_into(
        &self,
        input: FloatingDynamicWbcInput<'_>,
        output: &mut FloatingDynamicWbcOutput,
        scratch: &mut FloatingDynamicWbcScratch,
    ) -> Result<(), DynamicWbcError> {
        self.solve_into_impl(input, None, None, output, scratch)
    }

    /// Solve while conservatively propagating a reconstructed-state error
    /// envelope into support, self-collision, and world-collision hard rows.
    /// The caller must also provide generalized acceleration bounds already
    /// robustified for joint position/velocity uncertainty.
    pub fn solve_into_with_observation_error(
        &self,
        input: FloatingDynamicWbcInput<'_>,
        observation_error: RobotObservationErrorBound,
        output: &mut FloatingDynamicWbcOutput,
        scratch: &mut FloatingDynamicWbcScratch,
    ) -> Result<(), DynamicWbcError> {
        self.solve_into_impl(input, None, Some(observation_error), output, scratch)
    }

    /// Solve the floating constrained-acceleration problem with generalized
    /// actuator effort supplied rather than optimized.
    ///
    /// The effort term is moved into the dynamics right-hand side, avoiding
    /// numerically degenerate lower-equals-upper torque bounds. This is a
    /// state-local realization query: it does not integrate the returned
    /// acceleration or mutate contact policy.
    pub fn solve_with_fixed_generalized_effort_into(
        &self,
        input: FloatingDynamicWbcInput<'_>,
        fixed_generalized_effort: &DVector<f64>,
        output: &mut FloatingDynamicWbcOutput,
        scratch: &mut FloatingDynamicWbcScratch,
    ) -> Result<(), DynamicWbcError> {
        self.solve_into_impl(input, Some(fixed_generalized_effort), None, output, scratch)
    }

    fn solve_into_impl(
        &self,
        input: FloatingDynamicWbcInput<'_>,
        fixed_generalized_effort: Option<&DVector<f64>>,
        observation_error: Option<RobotObservationErrorBound>,
        output: &mut FloatingDynamicWbcOutput,
        scratch: &mut FloatingDynamicWbcScratch,
    ) -> Result<(), DynamicWbcError> {
        let dof = self.model.dof;
        let generalized_dof = dof + 6;
        let contacts = input.contacts.len();
        if contacts > scratch.maximum_contacts {
            return Err(DynamicWbcError::ContactCapacity {
                capacity: scratch.maximum_contacts,
                actual: contacts,
            });
        }
        validate_actuator_effort_input(
            input.actuator_effort,
            &self.model,
            scratch.maximum_actuators,
        )?;
        input.state.validate(&self.model)?;
        if !input
            .root_twist_world
            .0
            .iter()
            .all(|value| value.is_finite())
            || input.desired_generalized_acceleration.len() != generalized_dof
            || !input
                .desired_generalized_acceleration
                .iter()
                .all(|value| value.is_finite())
            || !input.joint_posture_weight.is_finite()
            || input.joint_posture_weight < 0.0
            || ![
                input.task_weights.root_angular,
                input.task_weights.root_horizontal,
                input.task_weights.root_height,
                input.task_weights.joint_posture,
            ]
            .into_iter()
            .all(|weight| weight.is_finite() && weight >= 0.0)
            || input.center_of_mass_task.is_some_and(|task| {
                !task
                    .desired_acceleration_world
                    .iter()
                    .all(|value| value.is_finite())
                    || !task.weight.is_finite()
                    || task.weight < 0.0
            })
            || input.joint_acceleration_task.is_some_and(|task| {
                task.coordinates.len() != task.desired_accelerations.len()
                    || task.coordinates.len() > dof
                    || !task.weight.is_finite()
                    || task.weight < 0.0
                    || task.coordinates.iter().any(|&coordinate| coordinate >= dof)
                    || task.coordinates.windows(2).any(|pair| pair[0] >= pair[1])
                    || !task
                        .desired_accelerations
                        .iter()
                        .all(|value| value.is_finite())
            })
            || input.frame_angular_acceleration_tasks.len() > FLOATING_ANGULAR_TASK_CAPACITY
            || input.frame_angular_acceleration_tasks.iter().any(|task| {
                task.stable_id < 9
                    || task.frame.0 >= self.model.bodies.len()
                    || !task
                        .desired_angular_acceleration_world
                        .iter()
                        .all(|value| value.is_finite())
                    || !task.weight.is_finite()
                    || task.weight < 0.0
            })
            || input
                .frame_angular_acceleration_tasks
                .windows(2)
                .any(|pair| pair[0].stable_id >= pair[1].stable_id)
            || input.point_acceleration_tasks.len() > FLOATING_POINT_TASK_CAPACITY
            || input.point_acceleration_tasks.iter().any(|task| {
                task.stable_id < 9
                    || task.frame.0 >= self.model.bodies.len()
                    || !task.point_in_frame.iter().all(|value| value.is_finite())
                    || !task
                        .desired_acceleration_world
                        .iter()
                        .all(|value| value.is_finite())
                    || !task.weight.is_finite()
                    || task.weight < 0.0
            })
            || input
                .point_acceleration_tasks
                .windows(2)
                .any(|pair| pair[0].stable_id >= pair[1].stable_id)
            || !input
                .generalized_acceleration_bounds
                .validate(generalized_dof)
            || !input.torque_bounds.validate(dof)
            || observation_error.is_some_and(|bound| !bound.validate())
            || fixed_generalized_effort.is_some_and(|effort| {
                effort.len() != dof
                    || input.actuator_effort.is_some()
                    || effort.iter().enumerate().any(|(coordinate, value)| {
                        !value.is_finite()
                            || *value < input.torque_bounds.lower[coordinate]
                            || *value > input.torque_bounds.upper[coordinate]
                    })
            })
            || output.generalized_acceleration.len() != generalized_dof
            || output.actuator_torque.len() != dof
            || output.contact_force_basis.ncols() != scratch.maximum_contacts
            || output.contact_force_world.ncols() != scratch.maximum_contacts
        {
            return Err(DynamicWbcError::InvalidInput);
        }
        for contact in input.contacts {
            if !contact.validate(&self.model) {
                return Err(DynamicWbcError::InvalidContact(contact.stable_id));
            }
        }
        if input
            .contacts
            .windows(2)
            .any(|pair| pair[0].stable_id >= pair[1].stable_id)
        {
            return Err(DynamicWbcError::InvalidInput);
        }
        validate_support_patches(input.contacts, input.support_patches)?;

        self.model
            .forward_kinematics(input.state, &mut scratch.model)?;
        self.model.floating_mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_matrix,
        )?;
        self.model.floating_bias_forces_into(
            input.state,
            input.root_twist_world,
            self.config.gravity_world,
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.bias,
        )?;
        if input.center_of_mass_task.is_some() {
            self.model.floating_com_jacobian_into(
                &scratch.model,
                &mut scratch.dynamics,
                &mut scratch.center_of_mass_jacobian,
            )?;
            scratch.center_of_mass_bias_acceleration_world = self
                .model
                .center_of_mass_bias_acceleration_world(&scratch.model, &scratch.dynamics)?;
        } else {
            scratch.center_of_mass_jacobian.fill(0.0);
            scratch.center_of_mass_bias_acceleration_world.fill(0.0);
        }

        let force_base = generalized_dof + usize::from(fixed_generalized_effort.is_none()) * dof;
        let variables = force_base + scratch.maximum_contacts * 3;
        prepare_floating_tasks(
            &mut scratch.tasks,
            dof,
            scratch.maximum_contacts,
            force_base,
            input,
            &self.config,
            &scratch.center_of_mass_jacobian,
            scratch.center_of_mass_bias_acceleration_world,
        );
        if fixed_generalized_effort.is_some() {
            // The supplied effort is a hard dynamics input, not another soft
            // regularization target over an otherwise unused decision block.
            scratch.tasks[8].weight = 0.0;
        }
        if input.centroidal_angular_momentum_task.is_some() {
            let center_of_mass_world = scratch.model.center_of_mass_world;
            let force_start = force_base;
            let task = &mut scratch.tasks[6];
            for (slot, contact) in input.contacts.iter().enumerate() {
                let point_world = scratch.model.world_from_body[contact.frame.0]
                    .transform_point(&Point3::from(contact.point_in_frame))
                    .coords;
                let moment_arm = point_world - center_of_mass_world;
                let basis = [
                    contact.tangent_x_world,
                    contact.tangent_y_world,
                    contact.normal_world,
                ];
                for (basis_axis, force_axis) in basis.iter().enumerate() {
                    let moment = moment_arm.cross(force_axis);
                    let column = force_start + slot * 3 + basis_axis;
                    for row in 0..3 {
                        task.jacobian[(row, column)] = moment[row];
                    }
                }
            }
        }
        for (slot, spec) in input
            .frame_angular_acceleration_tasks
            .iter()
            .copied()
            .enumerate()
        {
            self.model.floating_angular_jacobian_into(
                &scratch.model,
                spec.frame,
                &mut scratch.angular_task_jacobian,
            )?;
            let bias_acceleration = self
                .model
                .angular_bias_acceleration_world(spec.frame, &scratch.dynamics)?;
            let task = &mut scratch.tasks[9 + slot];
            task.stable_id = spec.stable_id;
            task.priority = spec.priority;
            task.weight = self.config.acceleration_weight * spec.weight;
            for row in 0..3 {
                for column in 0..generalized_dof {
                    task.jacobian[(row, column)] = scratch.angular_task_jacobian[(row, column)];
                }
                task.target_velocity[row] =
                    spec.desired_angular_acceleration_world[row] - bias_acceleration[row];
            }
        }
        for (slot, spec) in input.point_acceleration_tasks.iter().copied().enumerate() {
            self.model.floating_point_jacobian_into(
                &scratch.model,
                spec.frame,
                spec.point_in_frame,
                &mut scratch.point_task_jacobian,
            )?;
            let bias_acceleration = self.model.point_bias_acceleration_world(
                spec.frame,
                spec.point_in_frame,
                &scratch.model,
                &scratch.dynamics,
            )?;
            let task = &mut scratch.tasks[9 + FLOATING_ANGULAR_TASK_CAPACITY + slot];
            task.stable_id = spec.stable_id;
            task.priority = spec.priority;
            task.weight = self.config.acceleration_weight * spec.weight;
            for row in 0..3 {
                for column in 0..generalized_dof {
                    task.jacobian[(row, column)] = scratch.point_task_jacobian[(row, column)];
                }
                task.target_velocity[row] =
                    spec.desired_acceleration_world[row] - bias_acceleration[row];
            }
        }
        prepare_floating_bounds(
            &mut scratch.bounds,
            dof,
            scratch.maximum_contacts,
            force_base,
            input,
        );
        scratch.constraints.begin();
        emit_floating_dynamics_rows(
            &scratch.mass_matrix,
            &scratch.bias,
            dof,
            fixed_generalized_effort,
            &mut scratch.constraints,
        )?;
        if fixed_generalized_effort.is_none() {
            emit_actuator_effort_rows(
                &mut scratch.constraints,
                input.actuator_effort,
                generalized_dof,
            )?;
        }

        for (contact_index, contact) in input.contacts.iter().copied().enumerate() {
            self.model.floating_point_jacobian_into(
                &scratch.model,
                contact.frame,
                contact.point_in_frame,
                &mut scratch.contact_jacobian,
            )?;
            let bias_acceleration = self.model.point_bias_acceleration_world(
                contact.frame,
                contact.point_in_frame,
                &scratch.model,
                &scratch.dynamics,
            )?;
            augment_floating_dynamics_with_contact(
                &mut scratch.constraints,
                &scratch.contact_jacobian,
                contact,
                dof,
                force_base,
                contact_index,
            );
            emit_floating_contact_acceleration_rows(
                &mut scratch.constraints,
                &scratch.contact_jacobian,
                bias_acceleration,
                contact,
                generalized_dof,
                input.root_twist_world,
                input.state.v.as_slice(),
            )?;
            if fixed_generalized_effort.is_none() {
                emit_floating_friction_rows(
                    &mut scratch.constraints,
                    contact,
                    force_base,
                    contact_index,
                )?;
            }
        }
        if fixed_generalized_effort.is_none() {
            for patch in input.support_patches.iter().copied() {
                emit_floating_support_patch_rows(
                    &mut scratch.constraints,
                    input.contacts,
                    patch,
                    &scratch.model,
                    force_base,
                    observation_error.map_or(0.0, |bound| bound.center_of_mass_position_error_m),
                )?;
            }
        }
        if let (Some(collision), Some(config)) =
            (&self.collision, self.config.floating_collision_barrier)
        {
            let mut robust_config = config;
            robust_config.hard_margin += observation_error
                .map_or(0.0, |bound| 2.0 * bound.represented_point_position_error_m);
            collision.emit_floating_acceleration_barriers_into(
                &self.model,
                &scratch.model,
                &scratch.dynamics,
                input.root_twist_world,
                &input.state.v,
                robust_config,
                &mut scratch.constraints,
                &mut scratch.collision_barrier,
                &mut output.collision_barrier,
            )?;
        } else {
            output.collision_barrier = CollisionAccelerationBarrierEvidence::disabled();
        }
        if let (Some(world_collision), Some(config)) = (
            &self.world_collision,
            self.config.floating_world_collision_barrier,
        ) {
            let mut robust_config = config;
            robust_config.hard_margin +=
                observation_error.map_or(0.0, |bound| bound.represented_point_position_error_m);
            world_collision.emit_floating_acceleration_barriers_into(
                &self.model,
                &scratch.model,
                &scratch.dynamics,
                input.root_twist_world,
                &input.state.v,
                robust_config,
                &mut scratch.constraints,
                &mut scratch.world_collision_barrier,
                &mut output.world_collision_barrier,
            )?;
        } else {
            output.world_collision_barrier = WorldCollisionBarrierEvidence::disabled();
        }

        debug_assert_eq!(scratch.bounds.lower.len(), variables);
        self.solver.solve_constrained_into(
            variables,
            &scratch.tasks,
            &scratch.bounds,
            scratch.constraints.active(),
            &mut scratch.solve_result,
            &mut scratch.solver_workspace,
        );
        fill_floating_output(
            output,
            &scratch.solve_result,
            &scratch.tasks,
            &scratch.constraints,
            input,
            fixed_generalized_effort,
            dof,
            force_base,
            scratch.maximum_contacts,
        );
        if let Some(collision) = &self.collision {
            collision.finalize_floating_acceleration_barrier_evidence(
                &output.generalized_acceleration,
                &scratch.collision_barrier,
                &mut output.collision_barrier,
            )?;
        }
        if let Some(world_collision) = &self.world_collision {
            world_collision.finalize_floating_acceleration_barrier_evidence(
                &output.generalized_acceleration,
                &scratch.world_collision_barrier,
                &mut output.world_collision_barrier,
            )?;
        }
        Ok(())
    }
}

fn dynamic_task(
    rows: usize,
    variables: usize,
    stable_id: u32,
    kind: TaskKind,
    priority: Priority,
) -> Task {
    Task {
        stable_id,
        kind,
        priority,
        jacobian: DMatrix::zeros(rows, variables),
        target_velocity: DVector::zeros(rows),
        weight: 0.0,
    }
}

fn prepare_tasks(
    tasks: &mut [Task],
    dof: usize,
    maximum_contacts: usize,
    input: DynamicWbcInput<'_>,
    config: &DynamicWbcConfig,
) {
    for task in tasks.iter_mut() {
        task.jacobian.fill(0.0);
        task.target_velocity.fill(0.0);
    }
    let acceleration = &mut tasks[0];
    acceleration.weight = config.acceleration_weight;
    for coordinate in 0..dof {
        acceleration.jacobian[(coordinate, coordinate)] = 1.0;
        acceleration.target_velocity[coordinate] = input.desired_acceleration[coordinate];
    }

    let contact_force = &mut tasks[1];
    contact_force.weight = config.contact_force_weight;
    for slot in 0..maximum_contacts {
        for axis in 0..3 {
            let row = slot * 3 + axis;
            contact_force.jacobian[(row, 2 * dof + row)] = 1.0;
        }
    }
    for (slot, contact) in input.contacts.iter().enumerate() {
        contact_force.target_velocity[slot * 3] = contact.nominal_tangent_x_force;
        contact_force.target_velocity[slot * 3 + 1] = contact.nominal_tangent_y_force;
        contact_force.target_velocity[slot * 3 + 2] = contact.nominal_normal_force;
    }

    let torque = &mut tasks[2];
    torque.weight = config.actuator_torque_weight;
    if let Some(spec) = input.actuator_effort {
        for actuator in 0..dof {
            for coordinate in 0..dof {
                torque.jacobian[(actuator, dof + coordinate)] =
                    spec.actuation.generalized_from_actuator[(coordinate, actuator)];
            }
        }
    } else {
        for coordinate in 0..dof {
            torque.jacobian[(coordinate, dof + coordinate)] = 1.0;
        }
    }
}

fn prepare_floating_tasks(
    tasks: &mut [Task],
    dof: usize,
    maximum_contacts: usize,
    force_base: usize,
    input: FloatingDynamicWbcInput<'_>,
    config: &DynamicWbcConfig,
    center_of_mass_jacobian: &DMatrix<f64>,
    center_of_mass_bias_acceleration_world: Vec3,
) {
    let generalized_dof = dof + 6;
    for task in tasks.iter_mut() {
        task.jacobian.fill(0.0);
        task.target_velocity.fill(0.0);
    }
    let root_angular = &mut tasks[0];
    root_angular.priority = input.task_priorities.root_angular;
    root_angular.weight = config.acceleration_weight * input.task_weights.root_angular;
    for axis in 0..3 {
        root_angular.jacobian[(axis, axis)] = 1.0;
        root_angular.target_velocity[axis] = input.desired_generalized_acceleration[axis];
    }

    let root_horizontal = &mut tasks[1];
    root_horizontal.priority = input.task_priorities.root_horizontal;
    root_horizontal.weight = config.acceleration_weight * input.task_weights.root_horizontal;
    for axis in 0..2 {
        root_horizontal.jacobian[(axis, 3 + axis)] = 1.0;
        root_horizontal.target_velocity[axis] = input.desired_generalized_acceleration[3 + axis];
    }

    let root_height = &mut tasks[2];
    root_height.priority = input.task_priorities.root_height;
    root_height.weight = config.acceleration_weight * input.task_weights.root_height;
    root_height.jacobian[(0, 5)] = 1.0;
    root_height.target_velocity[0] = input.desired_generalized_acceleration[5];

    let joint_posture = &mut tasks[3];
    joint_posture.priority = input.task_priorities.joint_posture;
    joint_posture.weight =
        config.acceleration_weight * input.joint_posture_weight * input.task_weights.joint_posture;
    for coordinate in 0..dof {
        joint_posture.jacobian[(coordinate, 6 + coordinate)] = 1.0;
        joint_posture.target_velocity[coordinate] =
            input.desired_generalized_acceleration[6 + coordinate];
    }

    let joint_acceleration = &mut tasks[4];
    if let Some(spec) = input.joint_acceleration_task {
        joint_acceleration.priority = spec.priority;
        joint_acceleration.weight = config.acceleration_weight * spec.weight;
        for (row, (&coordinate, &target)) in spec
            .coordinates
            .iter()
            .zip(spec.desired_accelerations)
            .enumerate()
        {
            joint_acceleration.jacobian[(row, 6 + coordinate)] = 1.0;
            joint_acceleration.target_velocity[row] = target;
        }
    } else {
        joint_acceleration.weight = 0.0;
    }

    let center_of_mass = &mut tasks[5];
    if let Some(spec) = input.center_of_mass_task {
        center_of_mass.priority = spec.priority;
        center_of_mass.weight = config.acceleration_weight * spec.weight;
        let active_rows = if spec.horizontal_only { 2 } else { 3 };
        for row in 0..active_rows {
            for column in 0..generalized_dof {
                center_of_mass.jacobian[(row, column)] = center_of_mass_jacobian[(row, column)];
            }
            center_of_mass.target_velocity[row] =
                spec.desired_acceleration_world[row] - center_of_mass_bias_acceleration_world[row];
        }
        for row in active_rows..3 {
            center_of_mass.jacobian.row_mut(row).fill(0.0);
            center_of_mass.target_velocity[row] = 0.0;
        }
    } else {
        center_of_mass.weight = 0.0;
    }

    let centroidal_momentum = &mut tasks[6];
    if let Some(spec) = input.centroidal_angular_momentum_task {
        centroidal_momentum.priority = spec.priority;
        centroidal_momentum.weight = config.acceleration_weight * spec.weight;
        centroidal_momentum
            .target_velocity
            .as_mut_slice()
            .copy_from_slice(spec.desired_rate_world.as_slice());
    } else {
        centroidal_momentum.weight = 0.0;
    }

    let contact_force = &mut tasks[7];
    contact_force.weight = config.contact_force_weight;
    for slot in 0..maximum_contacts {
        for axis in 0..3 {
            let row = slot * 3 + axis;
            contact_force.jacobian[(row, force_base + row)] = 1.0;
        }
    }
    for (slot, contact) in input.contacts.iter().enumerate() {
        contact_force.target_velocity[slot * 3] = contact.nominal_tangent_x_force;
        contact_force.target_velocity[slot * 3 + 1] = contact.nominal_tangent_y_force;
        contact_force.target_velocity[slot * 3 + 2] = contact.nominal_normal_force;
    }

    let torque = &mut tasks[8];
    if force_base > generalized_dof {
        torque.weight = config.actuator_torque_weight;
        if let Some(spec) = input.actuator_effort {
            for actuator in 0..dof {
                for coordinate in 0..dof {
                    torque.jacobian[(actuator, generalized_dof + coordinate)] =
                        spec.actuation.generalized_from_actuator[(coordinate, actuator)];
                }
            }
        } else {
            for coordinate in 0..dof {
                torque.jacobian[(coordinate, generalized_dof + coordinate)] = 1.0;
            }
        }
    } else {
        torque.weight = 0.0;
    }
    for (slot, task) in tasks[9..].iter_mut().enumerate() {
        task.stable_id = 10 + slot as u32;
        task.priority = Priority::Intent;
        task.weight = 0.0;
    }
}

fn prepare_bounds(
    bounds: &mut VelocityBounds,
    dof: usize,
    maximum_contacts: usize,
    input: DynamicWbcInput<'_>,
) {
    for coordinate in 0..dof {
        bounds.lower[coordinate] = input.acceleration_bounds.lower[coordinate];
        bounds.upper[coordinate] = input.acceleration_bounds.upper[coordinate];
        bounds.lower[dof + coordinate] = input.torque_bounds.lower[coordinate];
        bounds.upper[dof + coordinate] = input.torque_bounds.upper[coordinate];
    }
    for slot in 0..maximum_contacts {
        let start = 2 * dof + slot * 3;
        if let Some(contact) = input.contacts.get(slot) {
            bounds.lower[start] = f64::NEG_INFINITY;
            bounds.upper[start] = f64::INFINITY;
            bounds.lower[start + 1] = f64::NEG_INFINITY;
            bounds.upper[start + 1] = f64::INFINITY;
            bounds.lower[start + 2] = contact.minimum_normal_force;
            bounds.upper[start + 2] = contact.maximum_normal_force;
        } else {
            for axis in 0..3 {
                bounds.lower[start + axis] = 0.0;
                bounds.upper[start + axis] = 0.0;
            }
        }
    }
}

fn prepare_floating_bounds(
    bounds: &mut VelocityBounds,
    dof: usize,
    maximum_contacts: usize,
    force_base: usize,
    input: FloatingDynamicWbcInput<'_>,
) {
    let generalized_dof = dof + 6;
    let fixed_effort_layout = force_base == generalized_dof;
    for coordinate in 0..generalized_dof {
        bounds.lower[coordinate] = if fixed_effort_layout {
            f64::NEG_INFINITY
        } else {
            input.generalized_acceleration_bounds.lower[coordinate]
        };
        bounds.upper[coordinate] = if fixed_effort_layout {
            f64::INFINITY
        } else {
            input.generalized_acceleration_bounds.upper[coordinate]
        };
    }
    if force_base > generalized_dof {
        for coordinate in 0..dof {
            bounds.lower[generalized_dof + coordinate] = input.torque_bounds.lower[coordinate];
            bounds.upper[generalized_dof + coordinate] = input.torque_bounds.upper[coordinate];
        }
    }
    for slot in 0..maximum_contacts {
        let start = force_base + slot * 3;
        if let Some(contact) = input.contacts.get(slot) {
            bounds.lower[start] = f64::NEG_INFINITY;
            bounds.upper[start] = f64::INFINITY;
            bounds.lower[start + 1] = f64::NEG_INFINITY;
            bounds.upper[start + 1] = f64::INFINITY;
            bounds.lower[start + 2] = if fixed_effort_layout {
                f64::NEG_INFINITY
            } else {
                contact.minimum_normal_force
            };
            bounds.upper[start + 2] = if fixed_effort_layout {
                f64::INFINITY
            } else {
                contact.maximum_normal_force
            };
        } else {
            for axis in 0..3 {
                bounds.lower[start + axis] = 0.0;
                bounds.upper[start + axis] = 0.0;
            }
        }
    }
}

fn validate_actuator_effort_input(
    input: Option<ActuatorEffortInput<'_>>,
    model: &CompiledModel,
    capacity: usize,
) -> Result<(), DynamicWbcError> {
    let Some(input) = input else {
        return Ok(());
    };
    let actuator_count = input.actuation.actuators.len();
    if actuator_count > capacity {
        return Err(DynamicWbcError::ActuatorCapacity {
            capacity,
            actual: actuator_count,
        });
    }
    if actuator_count != model.dof || input.actuation.actuator_from_generalized.is_none() {
        return Err(DynamicWbcError::UnsupportedActuationTopology {
            dof: model.dof,
            actuators: actuator_count,
        });
    }
    if input.actuation.validate(model).is_err() || !input.bounds.validate(actuator_count) {
        return Err(DynamicWbcError::InvalidInput);
    }
    Ok(())
}

fn emit_actuator_effort_rows(
    constraints: &mut ConstraintBuffer,
    input: Option<ActuatorEffortInput<'_>>,
    torque_start: usize,
) -> Result<(), DynamicWbcError> {
    let Some(input) = input else {
        return Ok(());
    };
    let dof = input.actuation.generalized_from_actuator.nrows();
    for actuator in 0..input.actuation.actuators.len() {
        let lower = input.bounds.lower[actuator];
        let upper = input.bounds.upper[actuator];
        if lower == f64::NEG_INFINITY && upper == f64::INFINITY {
            continue;
        }
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = ACTUATOR_EFFORT_ROW_BASE.saturating_add(actuator as u32);
        for coordinate in 0..dof {
            row.coefficients[torque_start + coordinate] =
                input.actuation.generalized_from_actuator[(coordinate, actuator)];
        }
        row.lower = lower;
        row.upper = upper;
    }
    Ok(())
}

fn actuator_effort_headroom(
    generalized_effort: &DVector<f64>,
    input: Option<ActuatorEffortInput<'_>>,
) -> (f64, Option<usize>) {
    let Some(input) = input else {
        return (f64::INFINITY, None);
    };
    let mut minimum_margin = f64::INFINITY;
    let mut limiting_actuator = None;
    for actuator in 0..input.actuation.actuators.len() {
        let effort = (0..generalized_effort.len())
            .map(|coordinate| {
                input.actuation.generalized_from_actuator[(coordinate, actuator)]
                    * generalized_effort[coordinate]
            })
            .sum::<f64>();
        let margin =
            (effort - input.bounds.lower[actuator]).min(input.bounds.upper[actuator] - effort);
        if margin < minimum_margin {
            minimum_margin = margin;
            limiting_actuator = Some(actuator);
        }
    }
    (minimum_margin, limiting_actuator)
}

fn emit_dynamics_rows(
    mass_matrix: &DMatrix<f64>,
    bias: &DVector<f64>,
    dof: usize,
    constraints: &mut ConstraintBuffer,
) -> Result<(), DynamicWbcError> {
    for equation in 0..dof {
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = DYNAMICS_ROW_BASE.saturating_add(equation as u32);
        for coordinate in 0..dof {
            row.coefficients[coordinate] = mass_matrix[(equation, coordinate)];
        }
        row.coefficients[dof + equation] = -1.0;
        row.lower = -bias[equation];
        row.upper = -bias[equation];
    }
    Ok(())
}

fn emit_floating_dynamics_rows(
    mass_matrix: &DMatrix<f64>,
    bias: &DVector<f64>,
    dof: usize,
    fixed_generalized_effort: Option<&DVector<f64>>,
    constraints: &mut ConstraintBuffer,
) -> Result<(), DynamicWbcError> {
    let generalized_dof = dof + 6;
    for equation in 0..generalized_dof {
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = DYNAMICS_ROW_BASE.saturating_add(equation as u32);
        for coordinate in 0..generalized_dof {
            row.coefficients[coordinate] = mass_matrix[(equation, coordinate)];
        }
        if equation >= 6 {
            if fixed_generalized_effort.is_none() {
                row.coefficients[generalized_dof + equation - 6] = -1.0;
            }
        }
        row.lower = -bias[equation]
            + fixed_generalized_effort.map_or(0.0, |effort| {
                if equation >= 6 {
                    effort[equation - 6]
                } else {
                    0.0
                }
            });
        row.upper = -bias[equation];
        row.upper = row.lower;
    }
    Ok(())
}

fn augment_dynamics_with_contact(
    constraints: &mut ConstraintBuffer,
    jacobian: &DMatrix<f64>,
    contact: ContactSpec,
    dof: usize,
    contact_index: usize,
) {
    let basis = [
        contact.tangent_x_world,
        contact.tangent_y_world,
        contact.normal_world,
    ];
    for equation in 0..dof {
        let row = &mut constraints.active_mut()[equation];
        for (axis_index, axis) in basis.iter().enumerate() {
            let mut generalized_force_coefficient = (0..3)
                .map(|world_axis| jacobian[(world_axis, equation)] * axis[world_axis])
                .sum::<f64>();
            if axis_index == 0
                && let ContactMode::RollingWheel {
                    coordinate,
                    velocity_coefficient,
                    ..
                } = contact.mode
                && equation == coordinate
            {
                generalized_force_coefficient += velocity_coefficient;
            }
            row.coefficients[2 * dof + contact_index * 3 + axis_index] =
                -generalized_force_coefficient;
        }
    }
}

fn augment_floating_dynamics_with_contact(
    constraints: &mut ConstraintBuffer,
    jacobian: &DMatrix<f64>,
    contact: ContactSpec,
    dof: usize,
    force_base: usize,
    contact_index: usize,
) {
    let generalized_dof = dof + 6;
    let force_start = force_base + contact_index * 3;
    let basis = [
        contact.tangent_x_world,
        contact.tangent_y_world,
        contact.normal_world,
    ];
    for equation in 0..generalized_dof {
        let row = &mut constraints.active_mut()[equation];
        for (axis_index, axis) in basis.iter().enumerate() {
            let mut generalized_force_coefficient = (0..3)
                .map(|world_axis| jacobian[(world_axis, equation)] * axis[world_axis])
                .sum::<f64>();
            if axis_index == 0
                && let ContactMode::RollingWheel {
                    coordinate,
                    velocity_coefficient,
                    ..
                } = contact.mode
                && equation == 6 + coordinate
            {
                generalized_force_coefficient += velocity_coefficient;
            }
            row.coefficients[force_start + axis_index] = -generalized_force_coefficient;
        }
    }
}

fn emit_contact_acceleration_rows(
    constraints: &mut ConstraintBuffer,
    jacobian: &DMatrix<f64>,
    bias_acceleration: Vec3,
    contact: ContactSpec,
    dof: usize,
    joint_velocity: &[f64],
) -> Result<(), DynamicWbcError> {
    if !contact.kinematic_enabled {
        return Ok(());
    }
    let axes: &[Vec3] = match contact.mode {
        ContactMode::LockedPoint => &[
            contact.tangent_x_world,
            contact.tangent_y_world,
            contact.normal_world,
        ],
        ContactMode::NormalPoint => &[contact.normal_world],
        ContactMode::RollingPoint => &[contact.tangent_y_world, contact.normal_world],
        ContactMode::RollingWheel { .. } => &[
            contact.tangent_x_world,
            contact.tangent_y_world,
            contact.normal_world,
        ],
    };
    for (axis_index, axis) in axes.iter().enumerate() {
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = CONTACT_ROW_BASE
            .saturating_add(contact.stable_id.saturating_mul(4))
            .saturating_add(axis_index as u32);
        for coordinate in 0..dof {
            row.coefficients[coordinate] = (0..3)
                .map(|world_axis| axis[world_axis] * jacobian[(world_axis, coordinate)])
                .sum();
        }
        let mut target = axis.dot(&(contact.desired_point_acceleration_world - bias_acceleration));
        if axis_index == 0
            && let ContactMode::RollingWheel {
                coordinate,
                velocity_coefficient,
                velocity_stabilization_gain,
                maximum_stabilization_acceleration,
            } = contact.mode
        {
            row.coefficients[coordinate] += velocity_coefficient;
            let point_velocity = (0..3)
                .map(|world_axis| {
                    axis[world_axis]
                        * (0..dof)
                            .map(|column| jacobian[(world_axis, column)] * joint_velocity[column])
                            .sum::<f64>()
                })
                .sum::<f64>();
            let rolling_velocity =
                point_velocity + velocity_coefficient * joint_velocity[coordinate];
            target += (-velocity_stabilization_gain * rolling_velocity).clamp(
                -maximum_stabilization_acceleration,
                maximum_stabilization_acceleration,
            );
        }
        row.lower = target;
        row.upper = target;
    }
    Ok(())
}

fn emit_floating_contact_acceleration_rows(
    constraints: &mut ConstraintBuffer,
    jacobian: &DMatrix<f64>,
    bias_acceleration: Vec3,
    contact: ContactSpec,
    generalized_dof: usize,
    root_twist_world: Motion6,
    joint_velocity: &[f64],
) -> Result<(), DynamicWbcError> {
    if !contact.kinematic_enabled {
        return Ok(());
    }
    let axes: &[Vec3] = match contact.mode {
        ContactMode::LockedPoint => &[
            contact.tangent_x_world,
            contact.tangent_y_world,
            contact.normal_world,
        ],
        ContactMode::NormalPoint => &[contact.normal_world],
        ContactMode::RollingPoint => &[contact.tangent_y_world, contact.normal_world],
        ContactMode::RollingWheel { .. } => &[
            contact.tangent_x_world,
            contact.tangent_y_world,
            contact.normal_world,
        ],
    };
    for (axis_index, axis) in axes.iter().enumerate() {
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = CONTACT_ROW_BASE
            .saturating_add(contact.stable_id.saturating_mul(4))
            .saturating_add(axis_index as u32);
        for coordinate in 0..generalized_dof {
            row.coefficients[coordinate] = (0..3)
                .map(|world_axis| axis[world_axis] * jacobian[(world_axis, coordinate)])
                .sum();
        }
        let mut target = axis.dot(&(contact.desired_point_acceleration_world - bias_acceleration));
        if axis_index == 0
            && let ContactMode::RollingWheel {
                coordinate,
                velocity_coefficient,
                velocity_stabilization_gain,
                maximum_stabilization_acceleration,
            } = contact.mode
        {
            row.coefficients[6 + coordinate] += velocity_coefficient;
            let point_velocity = (0..3)
                .map(|world_axis| {
                    axis[world_axis]
                        * ((0..6)
                            .map(|column| {
                                jacobian[(world_axis, column)] * root_twist_world.0[column]
                            })
                            .sum::<f64>()
                            + (6..generalized_dof)
                                .map(|column| {
                                    jacobian[(world_axis, column)] * joint_velocity[column - 6]
                                })
                                .sum::<f64>())
                })
                .sum::<f64>();
            let rolling_velocity =
                point_velocity + velocity_coefficient * joint_velocity[coordinate];
            target += (-velocity_stabilization_gain * rolling_velocity).clamp(
                -maximum_stabilization_acceleration,
                maximum_stabilization_acceleration,
            );
        }
        row.lower = target;
        row.upper = target;
    }
    Ok(())
}

fn validate_support_patches(
    contacts: &[ContactSpec],
    patches: &[SupportPatchSpec],
) -> Result<(), DynamicWbcError> {
    for (patch_index, patch) in patches.iter().copied().enumerate() {
        let end = patch.first_contact.saturating_add(patch.contact_count);
        let previous_end = patch_index.checked_sub(1).map(|index| {
            let previous = patches[index];
            previous
                .first_contact
                .saturating_add(previous.contact_count)
        });
        if patch.stable_id > 0x00ff_ffff
            || patch.contact_count < 3
            || patch.contact_count > SUPPORT_PATCH_POINT_CAPACITY
            || end > contacts.len()
            || !patch.minimum_margin_m.is_finite()
            || patch.minimum_margin_m < 0.0
            || !patch.minimum_total_normal_force.is_finite()
            || patch.minimum_total_normal_force < 0.0
            || previous_end.is_some_and(|value| value > patch.first_contact)
            || patch_index
                .checked_sub(1)
                .is_some_and(|index| patches[index].stable_id >= patch.stable_id)
        {
            return Err(DynamicWbcError::InvalidSupportPatch(patch.stable_id));
        }
        let patch_contacts = &contacts[patch.first_contact..end];
        let reference = patch_contacts[0];
        if patch_contacts.iter().skip(1).any(|contact| {
            (contact.tangent_x_world - reference.tangent_x_world).norm() > 1e-8
                || (contact.tangent_y_world - reference.tangent_y_world).norm() > 1e-8
                || (contact.normal_world - reference.normal_world).norm() > 1e-8
        }) {
            return Err(DynamicWbcError::InvalidSupportPatch(patch.stable_id));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct SupportHullPoint {
    xy: Vector2<f64>,
}

fn build_support_patch_hull(
    contacts: &[ContactSpec],
    patch: SupportPatchSpec,
    model: &ModelCache,
    hull: &mut [SupportHullPoint; SUPPORT_PATCH_POINT_CAPACITY * 2],
) -> Result<usize, DynamicWbcError> {
    const EPSILON: f64 = 1e-12;
    let reference = contacts[patch.first_contact];
    let mut sorted = [SupportHullPoint {
        xy: Vector2::new(0.0, 0.0),
    }; SUPPORT_PATCH_POINT_CAPACITY];
    let mut sorted_len = 0usize;
    for contact in contacts
        .iter()
        .skip(patch.first_contact)
        .take(patch.contact_count)
        .copied()
    {
        let point_world = model.world_from_body[contact.frame.0]
            .transform_point(&Point3::from(contact.point_in_frame))
            .coords;
        let point = SupportHullPoint {
            xy: Vector2::new(
                reference.tangent_x_world.dot(&point_world),
                reference.tangent_y_world.dot(&point_world),
            ),
        };
        let mut slot = sorted_len;
        while slot > 0
            && (sorted[slot - 1].xy.x > point.xy.x
                || (sorted[slot - 1].xy.x == point.xy.x && sorted[slot - 1].xy.y > point.xy.y))
        {
            sorted[slot] = sorted[slot - 1];
            slot -= 1;
        }
        sorted[slot] = point;
        sorted_len += 1;
    }
    let mut unique_len = 0usize;
    for slot in 0..sorted_len {
        if unique_len == 0 || (sorted[slot].xy - sorted[unique_len - 1].xy).norm() > EPSILON {
            sorted[unique_len] = sorted[slot];
            unique_len += 1;
        }
    }
    if unique_len < 3 {
        return Err(DynamicWbcError::InvalidSupportPatch(patch.stable_id));
    }

    let mut hull_len = 0usize;
    for point in sorted.iter().take(unique_len).copied() {
        while hull_len >= 2
            && cross2(
                hull[hull_len - 1].xy - hull[hull_len - 2].xy,
                point.xy - hull[hull_len - 1].xy,
            ) <= EPSILON
        {
            hull_len -= 1;
        }
        hull[hull_len] = point;
        hull_len += 1;
    }
    let upper_floor = hull_len + 1;
    for point in sorted.iter().take(unique_len - 1).rev().copied() {
        while hull_len >= upper_floor
            && cross2(
                hull[hull_len - 1].xy - hull[hull_len - 2].xy,
                point.xy - hull[hull_len - 1].xy,
            ) <= EPSILON
        {
            hull_len -= 1;
        }
        hull[hull_len] = point;
        hull_len += 1;
    }
    hull_len -= 1;
    if hull_len < 3 {
        return Err(DynamicWbcError::InvalidSupportPatch(patch.stable_id));
    }
    Ok(hull_len)
}

fn emit_floating_support_patch_rows(
    constraints: &mut ConstraintBuffer,
    contacts: &[ContactSpec],
    patch: SupportPatchSpec,
    model: &ModelCache,
    force_base: usize,
    margin_inflation_m: f64,
) -> Result<(), DynamicWbcError> {
    let mut hull = [SupportHullPoint {
        xy: Vector2::new(0.0, 0.0),
    }; SUPPORT_PATCH_POINT_CAPACITY * 2];
    let hull_len = build_support_patch_hull(contacts, patch, model, &mut hull)?;
    let reference = contacts[patch.first_contact];
    for edge_index in 0..hull_len {
        let start = hull[edge_index].xy;
        let edge = hull[(edge_index + 1) % hull_len].xy - start;
        let edge_length = edge.norm();
        if edge_length <= 1e-12 {
            return Err(DynamicWbcError::InvalidSupportPatch(patch.stable_id));
        }
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = SUPPORT_ROW_BASE | patch.stable_id.saturating_mul(16) | edge_index as u32;
        for (contact_index, contact) in contacts
            .iter()
            .enumerate()
            .skip(patch.first_contact)
            .take(patch.contact_count)
        {
            let point_world = model.world_from_body[contact.frame.0]
                .transform_point(&Point3::from(contact.point_in_frame))
                .coords;
            let point = Vector2::new(
                reference.tangent_x_world.dot(&point_world),
                reference.tangent_y_world.dot(&point_world),
            );
            let signed_distance = cross2(edge, point - start) / edge_length;
            row.coefficients[force_base + contact_index * 3 + 2] =
                signed_distance - patch.minimum_margin_m - margin_inflation_m;
        }
        row.lower = 0.0;
        row.upper = f64::INFINITY;
    }
    if patch.minimum_total_normal_force > 0.0 {
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = SUPPORT_ROW_BASE | patch.stable_id.saturating_mul(16) | hull_len as u32;
        for contact_index in patch.first_contact..patch.first_contact + patch.contact_count {
            row.coefficients[force_base + contact_index * 3 + 2] = 1.0;
        }
        row.lower = patch.minimum_total_normal_force;
        row.upper = f64::INFINITY;
    }
    Ok(())
}

fn emit_friction_rows(
    constraints: &mut ConstraintBuffer,
    contact: ContactSpec,
    dof: usize,
    contact_index: usize,
) -> Result<(), DynamicWbcError> {
    let start = 2 * dof + contact_index * 3;
    for (row_index, (axis, sign)) in [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0)]
        .into_iter()
        .enumerate()
    {
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = FRICTION_ROW_BASE
            .saturating_add(contact.stable_id.saturating_mul(4))
            .saturating_add(row_index as u32);
        row.coefficients[start + axis] = sign;
        row.coefficients[start + 2] = -contact.friction_coefficient;
        row.lower = f64::NEG_INFINITY;
        row.upper = 0.0;
    }
    Ok(())
}

fn emit_floating_friction_rows(
    constraints: &mut ConstraintBuffer,
    contact: ContactSpec,
    force_base: usize,
    contact_index: usize,
) -> Result<(), DynamicWbcError> {
    let start = force_base + contact_index * 3;
    for (row_index, (axis, sign)) in [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0)]
        .into_iter()
        .enumerate()
    {
        let row = constraints
            .push()
            .ok_or(DynamicWbcError::ConstraintCapacity)?;
        row.stable_id = FRICTION_ROW_BASE
            .saturating_add(contact.stable_id.saturating_mul(4))
            .saturating_add(row_index as u32);
        row.coefficients[start + axis] = sign;
        row.coefficients[start + 2] = -contact.friction_coefficient;
        row.lower = f64::NEG_INFINITY;
        row.upper = 0.0;
    }
    Ok(())
}

fn fill_output(
    output: &mut DynamicWbcOutput,
    solve: &SolveResult,
    constraints: &ConstraintBuffer,
    input: DynamicWbcInput<'_>,
    dof: usize,
    maximum_contacts: usize,
) {
    output.status = solve.diagnostics.status;
    output
        .generalized_acceleration
        .as_mut_slice()
        .copy_from_slice(&solve.velocity.as_slice()[..dof]);
    output
        .actuator_torque
        .as_mut_slice()
        .copy_from_slice(&solve.velocity.as_slice()[dof..2 * dof]);
    output.contact_force_basis.fill(0.0);
    output.contact_force_world.fill(0.0);
    output.active_contacts = input.contacts.len();
    output.minimum_friction_margin = f64::INFINITY;
    for slot in 0..maximum_contacts {
        let start = 2 * dof + slot * 3;
        for axis in 0..3 {
            output.contact_force_basis[(axis, slot)] = solve.velocity[start + axis];
        }
        if let Some(contact) = input.contacts.get(slot) {
            let force = contact.tangent_x_world * solve.velocity[start]
                + contact.tangent_y_world * solve.velocity[start + 1]
                + contact.normal_world * solve.velocity[start + 2];
            output.contact_force_world[(0, slot)] = force.x;
            output.contact_force_world[(1, slot)] = force.y;
            output.contact_force_world[(2, slot)] = force.z;
            output.minimum_friction_margin = output.minimum_friction_margin.min(
                contact.friction_coefficient * solve.velocity[start + 2]
                    - solve.velocity[start]
                        .abs()
                        .max(solve.velocity[start + 1].abs()),
            );
        }
    }
    if input.contacts.is_empty() {
        output.minimum_friction_margin = f64::INFINITY;
    }
    output.dynamics_residual_linf = constraints
        .active()
        .iter()
        .take(dof)
        .map(|row| {
            row.coefficients
                .iter()
                .zip(solve.velocity.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>()
                - row.lower
        })
        .map(f64::abs)
        .fold(0.0, f64::max);
    output.contact_acceleration_residual_linf = constraints
        .active()
        .iter()
        .filter(|row| row.stable_id & 0xf000_0000 == CONTACT_ROW_BASE)
        .map(|row| {
            row.coefficients
                .iter()
                .zip(solve.velocity.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>()
                - row.lower
        })
        .map(f64::abs)
        .fold(0.0, f64::max);
    output.minimum_torque_margin = (0..dof)
        .map(|coordinate| {
            let torque = output.actuator_torque[coordinate];
            (torque - input.torque_bounds.lower[coordinate])
                .min(input.torque_bounds.upper[coordinate] - torque)
        })
        .fold(f64::INFINITY, f64::min);
    (
        output.minimum_actuator_effort_margin,
        output.limiting_actuator,
    ) = actuator_effort_headroom(&output.actuator_torque, input.actuator_effort);
    output.solve.clone_from(&solve.diagnostics);
}

fn fill_floating_output(
    output: &mut FloatingDynamicWbcOutput,
    solve: &SolveResult,
    tasks: &[Task],
    constraints: &ConstraintBuffer,
    input: FloatingDynamicWbcInput<'_>,
    fixed_generalized_effort: Option<&DVector<f64>>,
    dof: usize,
    force_base: usize,
    maximum_contacts: usize,
) {
    let generalized_dof = dof + 6;
    output.status = solve.diagnostics.status;
    output
        .generalized_acceleration
        .as_mut_slice()
        .copy_from_slice(&solve.velocity.as_slice()[..generalized_dof]);
    if let Some(effort) = fixed_generalized_effort {
        output
            .actuator_torque
            .as_mut_slice()
            .copy_from_slice(effort.as_slice());
    } else {
        output
            .actuator_torque
            .as_mut_slice()
            .copy_from_slice(&solve.velocity.as_slice()[generalized_dof..force_base]);
    }
    output.contact_force_basis.fill(0.0);
    output.contact_force_world.fill(0.0);
    output.active_contacts = input.contacts.len();
    output.minimum_friction_margin = f64::INFINITY;
    for slot in 0..maximum_contacts {
        let start = force_base + slot * 3;
        for axis in 0..3 {
            output.contact_force_basis[(axis, slot)] = solve.velocity[start + axis];
        }
        if let Some(contact) = input.contacts.get(slot) {
            let force = contact.tangent_x_world * solve.velocity[start]
                + contact.tangent_y_world * solve.velocity[start + 1]
                + contact.normal_world * solve.velocity[start + 2];
            output.contact_force_world[(0, slot)] = force.x;
            output.contact_force_world[(1, slot)] = force.y;
            output.contact_force_world[(2, slot)] = force.z;
            output.minimum_friction_margin = output.minimum_friction_margin.min(
                contact.friction_coefficient * solve.velocity[start + 2]
                    - solve.velocity[start]
                        .abs()
                        .max(solve.velocity[start + 1].abs()),
            );
        }
    }
    if input.contacts.is_empty() {
        output.minimum_friction_margin = f64::INFINITY;
    }
    output.minimum_support_margin_m = f64::INFINITY;
    output.limiting_support_patch = None;
    for patch in input.support_patches {
        let total_normal_force = (patch.first_contact..patch.first_contact + patch.contact_count)
            .map(|contact_index| solve.velocity[force_base + contact_index * 3 + 2])
            .sum::<f64>();
        if total_normal_force <= 1e-9 {
            continue;
        }
        let stable_prefix = (SUPPORT_ROW_BASE | patch.stable_id.saturating_mul(16)) >> 4;
        for row in constraints
            .active()
            .iter()
            .filter(|row| row.stable_id >> 4 == stable_prefix)
        {
            let weighted_margin = row
                .coefficients
                .iter()
                .zip(solve.velocity.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>();
            let margin = patch.minimum_margin_m + weighted_margin / total_normal_force;
            if margin < output.minimum_support_margin_m {
                output.minimum_support_margin_m = margin;
                output.limiting_support_patch = Some(patch.stable_id);
            }
        }
    }
    output.dynamics_residual_linf = constraints
        .active()
        .iter()
        .take(generalized_dof)
        .map(|row| {
            row.coefficients
                .iter()
                .zip(solve.velocity.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>()
                - row.lower
        })
        .map(f64::abs)
        .fold(0.0, f64::max);
    output.contact_acceleration_residual_linf = constraints
        .active()
        .iter()
        .filter(|row| row.stable_id & 0xf000_0000 == CONTACT_ROW_BASE)
        .map(|row| {
            row.coefficients
                .iter()
                .zip(solve.velocity.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>()
                - row.lower
        })
        .map(f64::abs)
        .fold(0.0, f64::max);
    output.minimum_torque_margin = (0..dof)
        .map(|coordinate| {
            let torque = output.actuator_torque[coordinate];
            (torque - input.torque_bounds.lower[coordinate])
                .min(input.torque_bounds.upper[coordinate] - torque)
        })
        .fold(f64::INFINITY, f64::min);
    (
        output.minimum_actuator_effort_margin,
        output.limiting_actuator,
    ) = actuator_effort_headroom(&output.actuator_torque, input.actuator_effort);
    debug_assert_eq!(tasks.len(), output.task_residuals.len());
    for (slot, task) in output.task_residuals.iter_mut().zip(tasks) {
        let mut squared_residual = 0.0;
        let mut active_rows = 0usize;
        if task.weight > 0.0 {
            for row in 0..task.jacobian.nrows() {
                let jacobian_row = task.jacobian.row(row);
                if task.target_velocity[row] != 0.0
                    || jacobian_row.iter().any(|coefficient| *coefficient != 0.0)
                {
                    let residual = jacobian_row
                        .iter()
                        .zip(solve.velocity.iter())
                        .map(|(coefficient, value)| coefficient * value)
                        .sum::<f64>()
                        - task.target_velocity[row];
                    squared_residual += residual * residual;
                    active_rows += 1;
                }
            }
        }
        let l2 = squared_residual.sqrt();
        *slot = FloatingTaskResidual {
            stable_id: task.stable_id,
            kind: task.kind,
            priority: task.priority,
            active: active_rows > 0,
            clipped: solve.diagnostics.clipped_levels.contains(&task.priority),
            rows: active_rows,
            l2,
            rms: if active_rows > 0 {
                l2 / (active_rows as f64).sqrt()
            } else {
                0.0
            },
        };
    }
    output.solve.clone_from(&solve.diagnostics);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        urdf::load_urdf_file,
        world_collision::{
            CompiledWorldCollisionModel, DenseSdfGrid, SdfOutsidePolicy, SdfSampleSource,
            WorldSphereProbe,
        },
    };
    use nalgebra::{Translation3, UnitQuaternion};
    use std::path::PathBuf;

    fn upkie() -> CompiledModel {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/upkie/upkie.urdf");
        load_urdf_file(path).expect("pinned Upkie model")
    }

    fn toy_humanoid() -> CompiledModel {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/toy_humanoid.urdf");
        load_urdf_file(path).expect("repository toy humanoid")
    }

    fn tight_avoidance_toy() -> CompiledModel {
        let path =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/tight_avoidance_toy.urdf");
        load_urdf_file(path).expect("tight avoidance fixture")
    }

    #[test]
    fn floating_collision_barrier_overrides_inward_acceleration_and_reports_active_pair() {
        let model = tight_avoidance_toy();
        let barrier = CollisionAccelerationBarrierConfig {
            hard_margin: 0.02,
            influence_margin: 0.10,
            natural_frequency_rad_s: 10.0,
            damping_ratio: 1.0,
            ..CollisionAccelerationBarrierConfig::default()
        };
        let controller = FloatingDynamicWbc::new(
            model.clone(),
            DynamicWbcConfig {
                gravity_world: Vec3::zeros(),
                floating_collision_barrier: Some(barrier),
                ..DynamicWbcConfig::default()
            },
        )
        .unwrap();
        assert_eq!(controller.collision_pair_count(), 1);
        let mut state = RobotState::zeros(&model);
        state.q[0] = -0.12;
        state.v[0] = -0.2;
        let generalized_dof = model.dof + 6;
        let mut desired = DVector::zeros(generalized_dof);
        desired[6] = -50.0;
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -100.0),
            upper: DVector::from_element(model.dof, 100.0),
        };
        let mut scratch = FloatingDynamicWbcScratch::new(&model, 0);
        let mut output = FloatingDynamicWbcOutput::workspace(
            model.dof,
            0,
            controller.maximum_constraint_count(0, 0),
        );
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &state,
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
                &mut output,
                &mut scratch,
            )
            .unwrap();

        assert!(matches!(
            output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ));
        let evidence = output.collision_barrier;
        assert!(evidence.enabled);
        assert_eq!(evidence.represented_pair_count, 1);
        assert_eq!(evidence.active_pair_count, 1);
        assert_eq!(evidence.closest_pair_id, Some(0));
        assert_eq!(evidence.limiting_active_pair_id, Some(0));
        assert!((evidence.minimum_signed_distance_m - 0.03).abs() < 1e-12);
        assert!((evidence.limiting_relative_normal_velocity_mps + 0.2).abs() < 1e-12);
        assert!(evidence.limiting_required_normal_acceleration_mps2 > 2.9);
        assert!(evidence.limiting_achieved_normal_acceleration_mps2 >= 2.9);
        assert!(evidence.minimum_barrier_residual_mps2 >= -1e-8);
        assert!(output.generalized_acceleration[6] > 2.9);
        assert!(
            output
                .solve
                .active_constraints
                .contains(&barrier.hard_stable_id_base)
        );

        controller
            .solve_into_with_observation_error(
                FloatingDynamicWbcInput {
                    state: &state,
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
                RobotObservationErrorBound {
                    represented_point_position_error_m: 0.004,
                    ..RobotObservationErrorBound::default()
                },
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert!((evidence.minimum_margin_m - 0.010).abs() < 1e-12);
        assert!((output.collision_barrier.minimum_margin_m - 0.002).abs() < 1e-12);
        assert!(
            output
                .collision_barrier
                .limiting_required_normal_acceleration_mps2
                > evidence.limiting_required_normal_acceleration_mps2
        );
    }

    #[test]
    fn floating_world_sdf_barrier_overrides_wall_acceleration_and_reports_probe() {
        let model = tight_avoidance_toy();
        let dimensions = [5, 5, 5];
        let spacing = Vec3::repeat(0.5);
        let control_world_from_grid = crate::math::Transform3::from_parts(
            Translation3::new(-1.0, -1.0, -1.0),
            UnitQuaternion::identity(),
        );
        let mut samples = Vec::new();
        for _z in 0..dimensions[2] {
            for y in 0..dimensions[1] {
                for _x in 0..dimensions[0] {
                    let world_y = -1.0 + y as f64 * spacing.y;
                    samples.push(0.55 - world_y);
                }
            }
        }
        let field = DenseSdfGrid::new(
            control_world_from_grid,
            dimensions,
            spacing,
            samples,
            SdfOutsidePolicy::Reject,
        )
        .unwrap();
        let slider = model.body_id("slider").unwrap();
        let world_collision = CompiledWorldCollisionModel::new(
            &model,
            field,
            vec![WorldSphereProbe {
                stable_id: 17,
                body: slider,
                body_from_sphere: crate::math::Transform3::identity(),
                radius_m: 0.1,
                proxy_quality: crate::collision::DistanceQuality::ExactSphere,
            }],
            0,
        )
        .unwrap();
        let barrier = CollisionAccelerationBarrierConfig {
            hard_margin: 0.02,
            influence_margin: 0.10,
            natural_frequency_rad_s: 10.0,
            damping_ratio: 1.0,
            hard_stable_id_base: 0x7000_0000,
        };
        let controller = FloatingDynamicWbc::new_with_world_collision(
            model.clone(),
            DynamicWbcConfig {
                gravity_world: Vec3::zeros(),
                floating_world_collision_barrier: Some(barrier),
                ..DynamicWbcConfig::default()
            },
            world_collision,
        )
        .unwrap();
        assert_eq!(controller.collision_pair_count(), 0);
        assert_eq!(controller.world_collision_probe_count(), 1);
        assert_eq!(controller.world_collision_probe_body(17), Some(slider));

        let mut state = RobotState::zeros(&model);
        state.q[0] = 0.10;
        state.v[0] = 0.20;
        let generalized_dof = model.dof + 6;
        let mut desired = DVector::zeros(generalized_dof);
        desired[6] = 50.0;
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -100.0),
            upper: DVector::from_element(model.dof, 100.0),
        };
        let mut scratch = controller.scratch(0, 0);
        let mut output = FloatingDynamicWbcOutput::workspace(
            model.dof,
            0,
            controller.maximum_constraint_count(0, 0),
        );
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &state,
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
                &mut output,
                &mut scratch,
            )
            .unwrap();

        assert!(matches!(
            output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ));
        assert!(!output.collision_barrier.enabled);
        let evidence = output.world_collision_barrier;
        assert!(evidence.enabled);
        assert_eq!(evidence.represented_probe_count, 1);
        assert_eq!(evidence.active_probe_count, 1);
        assert_eq!(evidence.closest_probe_id, Some(17));
        assert_eq!(evidence.closest_body, Some(slider));
        assert_eq!(
            evidence.closest_field_source,
            Some(SdfSampleSource::TrilinearGrid)
        );
        assert_eq!(evidence.limiting_active_probe_id, Some(17));
        assert!((evidence.minimum_signed_distance_m - 0.05).abs() < 1e-12);
        assert!((evidence.limiting_relative_normal_velocity_mps + 0.20).abs() < 1e-12);
        assert!((evidence.closest_gradient_norm - 1.0).abs() < 1e-12);
        assert!(evidence.limiting_required_normal_acceleration_mps2 > 0.99);
        assert!(evidence.limiting_achieved_normal_acceleration_mps2 >= 0.99);
        assert!(evidence.minimum_barrier_residual_mps2 >= -1e-8);
        assert!(output.generalized_acceleration[6] < -0.99);
        assert!(
            output
                .solve
                .active_constraints
                .contains(&(barrier.hard_stable_id_base + 17))
        );
    }

    #[test]
    fn joint_acceleration_interval_brakes_before_a_position_limit() {
        let unrestricted = joint_acceleration_interval(0.0, 0.0, -0.5, 0.5, 8.0, 200.0, 0.005)
            .expect("valid interval");
        assert_eq!(unrestricted, (-200.0, 200.0));

        let near_lower =
            joint_acceleration_interval(-0.255, -1.8, -0.2618, 0.2618, 8.0, 200.0, 0.005)
                .expect("valid interval");
        assert!(near_lower.0 > 0.0, "unsafe outward speed must brake");
        assert!(near_lower.0 <= near_lower.1);

        let already_unsafe =
            joint_acceleration_interval(-0.239, -6.0, -0.2618, 0.2618, 8.0, 200.0, 0.005)
                .expect("emergency interval");
        assert_eq!(already_unsafe, (200.0, 200.0));
    }

    #[test]
    fn joint_acceleration_interval_supports_continuous_coordinates() {
        assert_eq!(
            joint_acceleration_interval(
                12.0,
                1.0,
                f64::NEG_INFINITY,
                f64::INFINITY,
                8.0,
                200.0,
                0.005,
            ),
            Some((-200.0, 200.0))
        );
    }

    #[test]
    fn observation_error_conservatively_intersects_joint_stopping_intervals() {
        let nominal =
            joint_acceleration_interval(0.23, 3.5, -0.2618, 0.2618, 8.0, 200.0, 0.005).unwrap();
        let exact = joint_acceleration_interval_with_observation_error(
            0.23, 3.5, 0.0, 0.0, -0.2618, 0.2618, 8.0, 200.0, 0.005,
        )
        .unwrap();
        let robust = joint_acceleration_interval_with_observation_error(
            0.23, 3.5, 0.002, 0.05, -0.2618, 0.2618, 8.0, 200.0, 0.005,
        )
        .unwrap();
        assert_eq!(exact, nominal);
        assert!(robust.0 >= nominal.0);
        assert!(robust.1 < nominal.1);
        assert_eq!(
            joint_acceleration_interval_with_observation_error(
                0.0, 0.0, -0.1, 0.0, -1.0, 1.0, 8.0, 200.0, 0.005,
            ),
            None
        );
    }

    #[test]
    fn joint_velocity_envelope_is_inactive_then_smoothly_brakes() {
        assert_eq!(
            joint_velocity_envelope_acceleration(5.9, 8.0, 0.75, 10.0, 200.0),
            Some(0.0)
        );
        let positive = joint_velocity_envelope_acceleration(7.0, 8.0, 0.75, 10.0, 200.0).unwrap();
        let negative = joint_velocity_envelope_acceleration(-7.0, 8.0, 0.75, 10.0, 200.0).unwrap();
        assert!(positive < 0.0 && positive > -80.0);
        assert_eq!(negative, -positive);
        assert_eq!(
            joint_velocity_envelope_acceleration(8.0, 8.0, 0.75, 30.0, 100.0),
            Some(-100.0)
        );
        assert_eq!(
            joint_velocity_envelope_acceleration(1.0, 0.0, 0.75, 10.0, 100.0),
            None
        );
    }

    #[test]
    fn joint_position_capture_tracks_directional_stopping_headroom() {
        assert_eq!(
            joint_position_capture_acceleration(0.0, 1.0, -1.0, 1.0, 50.0, 0.02, 200.0),
            Some(0.0)
        );
        let upper =
            joint_position_capture_acceleration(0.98, 2.0, -1.0, 1.0, 50.0, 0.02, 200.0).unwrap();
        let lower =
            joint_position_capture_acceleration(-0.98, -2.0, -1.0, 1.0, 50.0, 0.02, 200.0).unwrap();
        assert!(upper < 0.0 && upper >= -200.0);
        assert_eq!(lower, -upper);
        assert_eq!(
            joint_position_capture_acceleration(0.0, 0.0, -1.0, 1.0, 50.0, 0.02, 200.0),
            Some(0.0)
        );
        assert_eq!(
            joint_position_capture_acceleration(0.0, 1.0, 1.0, -1.0, 50.0, 0.02, 200.0),
            None
        );
    }

    #[test]
    fn contact_phase_authority_uses_measured_support_before_precontact_intent() {
        let config = ContactPhaseAuthorityConfig {
            unsupported_joint_velocity_scale: 0.0,
            single_support_joint_velocity_scale: 0.1,
            precontact_joint_velocity_scale: 0.2,
            multi_support_joint_velocity_scale: 1.0,
        };
        for (supports, precontact, phase, scale) in [
            (0, false, MeasuredContactPhase::Unsupported, 0.0),
            (1, false, MeasuredContactPhase::SingleSupport, 0.1),
            (1, true, MeasuredContactPhase::Precontact, 0.2),
            (2, true, MeasuredContactPhase::MultiSupport, 1.0),
        ] {
            let output = contact_phase_authority(supports, precontact, config).unwrap();
            assert_eq!(output.phase, phase);
            assert_eq!(output.joint_velocity_envelope_scale, scale);
        }
        assert!(
            contact_phase_authority(
                1,
                false,
                ContactPhaseAuthorityConfig {
                    single_support_joint_velocity_scale: f64::NAN,
                    ..config
                }
            )
            .is_none()
        );
    }

    #[test]
    fn contact_phase_authority_slew_is_bounded_and_does_not_overshoot() {
        assert!((slew_contact_phase_authority(0.2, 1.0, 0.1).unwrap() - 0.3).abs() < 1e-12);
        assert_eq!(slew_contact_phase_authority(0.95, 1.0, 0.1), Some(1.0));
        assert!((slew_contact_phase_authority(0.8, 0.0, 0.25).unwrap() - 0.55).abs() < 1e-12);
        assert_eq!(slew_contact_phase_authority(0.1, 0.0, 0.25), Some(0.0));
        assert_eq!(slew_contact_phase_authority(0.0, 1.0, 0.0), None);
    }

    #[test]
    fn balance_feedback_authority_blends_capture_attitude_and_landing_age() {
        let config = BalanceFeedbackAuthorityConfig::default();
        let full = balance_feedback_authority(-0.01, 0.0, 0, config).unwrap();
        assert_eq!(full.joint_velocity_envelope_scale, 1.0);
        assert_eq!(full.dcm_margin_scale, 1.0);

        let midpoint =
            balance_feedback_authority(0.015, 17.5_f64.to_radians(), 200, config).unwrap();
        assert!((midpoint.dcm_margin_scale - 0.5).abs() < 1e-12);
        assert!((midpoint.attitude_scale - 0.5).abs() < 1e-12);
        assert!((midpoint.precontact_age_scale - 0.5).abs() < 1e-12);
        assert!((midpoint.joint_velocity_envelope_scale - 0.125).abs() < 1e-12);

        assert_eq!(
            balance_feedback_authority(-0.01, 30.0_f64.to_radians(), 0, config)
                .unwrap()
                .joint_velocity_envelope_scale,
            0.0
        );
        assert_eq!(
            balance_feedback_authority(-0.01, 0.0, 250, config)
                .unwrap()
                .joint_velocity_envelope_scale,
            0.0
        );
        assert!(balance_feedback_authority(f64::NAN, 0.0, 0, config).is_none());
    }

    #[test]
    fn dcm_balance_closes_error_through_an_unclipped_virtual_zmp() {
        let support = [
            Vec3::new(-0.5, -0.5, 0.0),
            Vec3::new(-0.5, 0.5, 0.0),
            Vec3::new(0.5, -0.5, 0.0),
            Vec3::new(0.5, 0.5, 0.0),
        ];
        let config = DcmBalanceConfig {
            support_margin_m: 0.0,
            ..DcmBalanceConfig::default()
        };
        let output = dcm_balance_acceleration(
            Vec3::new(0.1, 0.0, 1.0),
            Vec3::zeros(),
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::zeros(),
            &support,
            config,
        )
        .expect("valid DCM command");
        let omega = 9.81_f64.sqrt();
        assert!((output.natural_frequency_rad_s - omega).abs() < 1e-12);
        assert!(!output.zmp_was_clipped);
        assert!((output.virtual_zmp_world.x - (0.1 + 0.35 / omega)).abs() < 1e-12);
        assert!(output.desired_horizontal_acceleration_world.x < 0.0);
        assert_eq!(output.support_vertex_count, 4);
        assert!((output.dcm_support_margin_m - 0.4).abs() < 1e-12);
    }

    #[test]
    fn dcm_balance_projects_to_the_eroded_support_polygon() {
        let support = [
            Vec3::new(-0.5, -0.5, 0.0),
            Vec3::new(-0.5, 0.5, 0.0),
            Vec3::new(0.5, -0.5, 0.0),
            Vec3::new(0.5, 0.5, 0.0),
        ];
        let output = dcm_balance_acceleration(
            Vec3::new(0.45, 0.45, 1.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::zeros(),
            Vec3::zeros(),
            &support,
            DcmBalanceConfig {
                support_margin_m: 0.1,
                ..DcmBalanceConfig::default()
            },
        )
        .expect("valid eroded support polygon");
        assert!(output.zmp_was_clipped);
        assert!((output.clipped_zmp_world.x - 0.4).abs() < 1e-12);
        assert!((output.clipped_zmp_world.y - 0.4).abs() < 1e-12);
        assert!(output.dcm_support_margin_m < 0.0);
    }

    #[test]
    fn dcm_balance_rejects_an_empty_margin_set() {
        let support = [
            Vec3::new(-0.1, -0.02, 0.0),
            Vec3::new(-0.1, 0.02, 0.0),
            Vec3::new(0.1, -0.02, 0.0),
            Vec3::new(0.1, 0.02, 0.0),
        ];
        assert!(
            dcm_balance_acceleration(
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::zeros(),
                Vec3::zeros(),
                Vec3::zeros(),
                &support,
                DcmBalanceConfig {
                    support_margin_m: 0.03,
                    ..DcmBalanceConfig::default()
                },
            )
            .is_none()
        );
    }

    #[test]
    fn dcm_balance_keeps_failure_traces_defined_with_a_height_floor() {
        let support = [
            Vec3::new(-0.5, -0.5, 0.0),
            Vec3::new(-0.5, 0.5, 0.0),
            Vec3::new(0.5, -0.5, 0.0),
            Vec3::new(0.5, 0.5, 0.0),
        ];
        let output = dcm_balance_acceleration(
            Vec3::new(0.0, 0.0, -0.1),
            Vec3::zeros(),
            Vec3::zeros(),
            Vec3::zeros(),
            &support,
            DcmBalanceConfig::default(),
        )
        .expect("fallen traces remain observable");
        assert_eq!(output.measured_com_height_m, -0.1);
        assert_eq!(output.used_com_height_m, 0.2);
        assert!(output.com_height_was_clamped);
    }

    #[test]
    fn normal_point_emits_only_the_physical_normal_row() {
        let generalized_dof = 8;
        let mut constraints = ConstraintBuffer::new(generalized_dof, 3);
        constraints.begin();
        let jacobian = DMatrix::from_row_slice(
            3,
            generalized_dof,
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0,
                -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0,
            ],
        );
        let mut contact = ContactSpec::horizontal(
            7,
            FrameId(0),
            Vec3::zeros(),
            ContactMode::NormalPoint,
            0.8,
            1_000.0,
            100.0,
        );
        contact.desired_point_acceleration_world = Vec3::new(0.4, -0.2, 1.5);
        emit_floating_contact_acceleration_rows(
            &mut constraints,
            &jacobian,
            Vec3::new(0.1, 0.2, 0.3),
            contact,
            generalized_dof,
            Motion6::default(),
            &[0.0, 0.0],
        )
        .unwrap();
        assert_eq!(constraints.active_len(), 1);
        let row = &constraints.active()[0];
        for coordinate in 0..generalized_dof {
            assert_eq!(row.coefficients[coordinate], jacobian[(2, coordinate)]);
        }
        assert!((row.lower - 1.2).abs() < 1e-12);
        assert_eq!(row.lower, row.upper);
    }

    #[test]
    fn support_patch_can_keep_four_force_points_with_six_kinematic_rows() {
        let generalized_dof = 6;
        let jacobian = DMatrix::zeros(3, generalized_dof);
        let mut constraints = ConstraintBuffer::new(generalized_dof, 12);
        constraints.begin();
        let modes = [
            (ContactMode::LockedPoint, true),
            (ContactMode::NormalPoint, true),
            (ContactMode::LockedPoint, false),
            (ContactMode::RollingPoint, true),
        ];
        for (point, (mode, kinematic_enabled)) in modes.into_iter().enumerate() {
            let mut contact = ContactSpec::horizontal(
                1 + point as u32,
                FrameId(0),
                Vec3::zeros(),
                mode,
                1.0,
                1_000.0,
                100.0,
            );
            contact.kinematic_enabled = kinematic_enabled;
            emit_floating_contact_acceleration_rows(
                &mut constraints,
                &jacobian,
                Vec3::zeros(),
                contact,
                generalized_dof,
                Motion6::default(),
                &[],
            )
            .unwrap();
        }
        assert_eq!(constraints.active_len(), 6);
    }

    #[test]
    fn finite_support_patch_rows_enforce_an_eroded_cop_polygon_without_allocating() {
        let model = toy_humanoid();
        let state = RobotState::zeros(&model);
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let foot = model.frame_id("left_foot").unwrap();
        let points = [
            Vec3::new(-0.08, -0.04, -0.07),
            Vec3::new(-0.08, 0.04, -0.07),
            Vec3::new(0.12, -0.04, -0.07),
            Vec3::new(0.12, 0.04, -0.07),
        ];
        let contacts = points.map(|point| {
            ContactSpec::horizontal(
                1,
                foot,
                point,
                ContactMode::NormalPoint,
                0.8,
                1_000.0,
                100.0,
            )
        });
        let patch = SupportPatchSpec {
            stable_id: 7,
            first_contact: 0,
            contact_count: contacts.len(),
            minimum_margin_m: 0.02,
            minimum_total_normal_force: 0.0,
        };
        let force_base = model.dof + 6 + model.dof;
        let variables = force_base + contacts.len() * 3;
        let mut constraints = ConstraintBuffer::new(variables, 4);
        constraints.begin();
        emit_floating_support_patch_rows(
            &mut constraints,
            &contacts,
            patch,
            &cache,
            force_base,
            0.0,
        )
        .unwrap();
        assert_eq!(constraints.active_len(), 4);

        let mut centered_load = DVector::zeros(variables);
        for contact in 0..contacts.len() {
            centered_load[force_base + contact * 3 + 2] = 25.0;
        }
        let evaluate = |row: &crate::solver::LinearConstraint, load: &DVector<f64>| {
            row.coefficients
                .iter()
                .zip(load.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>()
        };
        assert!(constraints.active().iter().all(|row| {
            evaluate(row, &centered_load) >= -1e-12
                && row.stable_id & 0xf000_0000 == SUPPORT_ROW_BASE
        }));

        let mut corner_load = DVector::zeros(variables);
        corner_load[force_base + 2] = 100.0;
        assert!(
            constraints
                .active()
                .iter()
                .any(|row| evaluate(row, &corner_load) < -1.0)
        );
    }

    #[test]
    fn finite_support_patch_can_require_an_aggregate_normal_load() {
        let model = toy_humanoid();
        let state = RobotState::zeros(&model);
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let foot = model.frame_id("left_foot").unwrap();
        let points = [
            Vec3::new(-0.08, -0.04, -0.07),
            Vec3::new(-0.08, 0.04, -0.07),
            Vec3::new(0.12, -0.04, -0.07),
            Vec3::new(0.12, 0.04, -0.07),
        ];
        let contacts = points.map(|point| {
            ContactSpec::horizontal(
                1,
                foot,
                point,
                ContactMode::NormalPoint,
                0.8,
                1_000.0,
                100.0,
            )
        });
        let patch = SupportPatchSpec {
            stable_id: 8,
            first_contact: 0,
            contact_count: contacts.len(),
            minimum_margin_m: 0.0,
            minimum_total_normal_force: 80.0,
        };
        let force_base = model.dof + 6 + model.dof;
        let variables = force_base + contacts.len() * 3;
        let mut constraints = ConstraintBuffer::new(variables, 5);
        constraints.begin();
        emit_floating_support_patch_rows(
            &mut constraints,
            &contacts,
            patch,
            &cache,
            force_base,
            0.0,
        )
        .unwrap();
        assert_eq!(constraints.active_len(), 5);
        let floor = constraints
            .active()
            .iter()
            .find(|row| row.stable_id == (SUPPORT_ROW_BASE | 8 * 16 | 4))
            .expect("aggregate support-load row");
        let mut load = DVector::zeros(variables);
        for contact in 0..contacts.len() {
            load[force_base + contact * 3 + 2] = 20.0;
        }
        let evaluate = |row: &crate::solver::LinearConstraint, load: &DVector<f64>| {
            row.coefficients
                .iter()
                .zip(load.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>()
        };
        assert!((evaluate(floor, &load) - 80.0).abs() < 1e-12);
        load[force_base + 2] = 19.0;
        assert!(evaluate(floor, &load) < 80.0);
    }

    #[test]
    fn official_g1_rank_minimal_sole_lock_has_six_independent_rows_when_cached() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf");
        if !path.exists() {
            return;
        }
        let model = load_urdf_file(path).expect("pinned Unitree G1 model");
        let mut state = RobotState::zeros(&model);
        for (name, value) in [
            ("left_hip_pitch_joint", -0.1),
            ("left_knee_joint", 0.3),
            ("left_ankle_pitch_joint", -0.2),
            ("right_hip_pitch_joint", -0.1),
            ("right_knee_joint", 0.3),
            ("right_ankle_pitch_joint", -0.2),
        ] {
            let joint = model.joint_id(name).unwrap();
            state.q[model.joints[joint.0].coordinate.unwrap()] = value;
        }
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let frame = model.frame_id("left_ankle_roll_link").unwrap();
        let points = [
            Vec3::new(-0.05, -0.0275, -0.035),
            Vec3::new(-0.05, 0.0275, -0.035),
            Vec3::new(0.12, -0.0275, -0.035),
            Vec3::new(0.12, 0.0275, -0.035),
        ];
        let generalized_dof = model.dof + 6;
        let mut point_jacobian = DMatrix::zeros(3, generalized_dof);
        let mut patch_jacobian = DMatrix::zeros(6, generalized_dof);
        let mut row = 0;
        for (point, axes) in [(0, &[0, 1, 2][..]), (1, &[2][..]), (3, &[1, 2][..])] {
            model
                .floating_point_jacobian_into(&cache, frame, points[point], &mut point_jacobian)
                .unwrap();
            for axis in axes {
                patch_jacobian
                    .row_mut(row)
                    .copy_from(&point_jacobian.row(*axis));
                row += 1;
            }
        }
        let singular_values = patch_jacobian.svd(false, false).singular_values;
        assert_eq!(
            singular_values
                .iter()
                .filter(|value| **value > 1e-8)
                .count(),
            6
        );
        assert!(singular_values.min() > 1e-3);
    }

    #[test]
    fn unified_dynamic_solve_enforces_contacts_friction_and_rigid_body_equation() {
        let model = upkie();
        let controller = DynamicWbc::new(model.clone(), DynamicWbcConfig::default()).unwrap();
        let state = RobotState::zeros(&model);
        let desired_acceleration = DVector::zeros(model.dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -100.0),
            upper: DVector::from_element(model.dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let total_weight: f64 = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
        let contacts = [
            ContactSpec::horizontal(
                1,
                model.frame_id("left_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
            ContactSpec::horizontal(
                2,
                model.frame_id("right_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
        ];
        let output = controller
            .solve(
                DynamicWbcInput {
                    state: &state,
                    desired_acceleration: &desired_acceleration,
                    acceleration_bounds: &acceleration_bounds,
                    torque_bounds: &torque_bounds,
                    actuator_effort: None,
                    contacts: &contacts,
                },
                2,
            )
            .unwrap();
        assert!(
            matches!(
                output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ),
            "status {:?}, dynamics {}, contact {}, violation {}, active {:?}",
            output.status,
            output.dynamics_residual_linf,
            output.contact_acceleration_residual_linf,
            output.solve.maximum_constraint_violation,
            output.solve.active_constraints
        );
        assert!(output.dynamics_residual_linf < 1e-8);
        assert!(
            output.contact_acceleration_residual_linf < 1e-8,
            "contact residual {}, status {:?}, dynamics {}, solve violation {}",
            output.contact_acceleration_residual_linf,
            output.status,
            output.dynamics_residual_linf,
            output.solve.maximum_constraint_violation
        );
        assert!(output.minimum_friction_margin >= -1e-9);
        assert!(output.minimum_torque_margin >= -1e-9);
        assert!(output.generalized_acceleration.norm() < 1e-8);
        assert!((output.contact_force_basis[(2, 0)] - 0.5 * total_weight).abs() < 1e-6);
        assert!((output.contact_force_basis[(2, 1)] - 0.5 * total_weight).abs() < 1e-6);
    }

    #[test]
    fn coupled_actuator_effort_polytope_clips_in_actuator_space() {
        let model = upkie();
        let controller = DynamicWbc::new(
            model.clone(),
            DynamicWbcConfig {
                actuator_torque_weight: 0.0,
                ..DynamicWbcConfig::default()
            },
        )
        .unwrap();
        let coordinate_names = model.coordinate_names();
        let left = coordinate_names
            .iter()
            .position(|name| name.contains("left_wheel"))
            .expect("left wheel coordinate");
        let right = coordinate_names
            .iter()
            .position(|name| name.contains("right_wheel"))
            .expect("right wheel coordinate");

        let mut actuation = CompiledActuation::identity_from_model(&model);
        actuation.generalized_from_actuator[(left, left)] = 0.5;
        actuation.generalized_from_actuator[(left, right)] = 0.5;
        actuation.generalized_from_actuator[(right, left)] = -1.0;
        actuation.generalized_from_actuator[(right, right)] = 1.0;
        let mut reverse = DMatrix::identity(model.dof, model.dof);
        reverse[(left, left)] = 1.0;
        reverse[(left, right)] = -0.5;
        reverse[(right, left)] = 1.0;
        reverse[(right, right)] = 0.5;
        actuation.actuator_from_generalized = Some(reverse);
        actuation.validate(&model).unwrap();

        let state = RobotState::zeros(&model);
        let mut desired_acceleration = DVector::zeros(model.dof);
        desired_acceleration[left] = 200.0;
        desired_acceleration[right] = -200.0;
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -200.0),
            upper: DVector::from_element(model.dof, 200.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let mut actuator_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        actuator_bounds.lower[left] = -0.02;
        actuator_bounds.upper[left] = 0.02;
        actuator_bounds.lower[right] = -0.02;
        actuator_bounds.upper[right] = 0.02;

        let unconstrained = controller
            .solve(
                DynamicWbcInput {
                    state: &state,
                    desired_acceleration: &desired_acceleration,
                    acceleration_bounds: &acceleration_bounds,
                    torque_bounds: &torque_bounds,
                    actuator_effort: None,
                    contacts: &[],
                },
                0,
            )
            .unwrap();
        let mut unconstrained_actuator_effort = DVector::zeros(model.dof);
        actuation
            .write_actuator_effort(
                &unconstrained.actuator_torque,
                &mut unconstrained_actuator_effort,
            )
            .unwrap();
        assert!(
            unconstrained_actuator_effort[left]
                .abs()
                .max(unconstrained_actuator_effort[right].abs())
                > 0.02 + 1e-6,
            "fixture must exceed the coupled actuator polytope: left {}, right {}",
            unconstrained_actuator_effort[left],
            unconstrained_actuator_effort[right]
        );

        let constrained = controller
            .solve(
                DynamicWbcInput {
                    state: &state,
                    desired_acceleration: &desired_acceleration,
                    acceleration_bounds: &acceleration_bounds,
                    torque_bounds: &torque_bounds,
                    actuator_effort: Some(ActuatorEffortInput {
                        actuation: &actuation,
                        bounds: &actuator_bounds,
                    }),
                    contacts: &[],
                },
                0,
            )
            .unwrap();
        let mut constrained_actuator_effort = DVector::zeros(model.dof);
        actuation
            .write_actuator_effort(
                &constrained.actuator_torque,
                &mut constrained_actuator_effort,
            )
            .unwrap();
        assert!(matches!(
            constrained.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ));
        assert!(constrained.dynamics_residual_linf < 1e-8);
        assert!(constrained.solve.maximum_constraint_violation < 1e-8);
        assert!(constrained.minimum_actuator_effort_margin >= -1e-8);
        assert!(constrained.minimum_actuator_effort_margin < 1e-5);
        assert!(constrained.limiting_actuator.is_some());
        assert!(constrained_actuator_effort[left].abs() <= 0.02 + 1e-8);
        assert!(constrained_actuator_effort[right].abs() <= 0.02 + 1e-8);
        assert!(
            constrained
                .solve
                .active_constraints
                .iter()
                .any(|stable_id| *stable_id & 0xf000_0000 == ACTUATOR_EFFORT_ROW_BASE)
        );
    }

    #[test]
    fn passive_actuation_topology_is_rejected_instead_of_box_approximated() {
        let model = upkie();
        let mut actuation = CompiledActuation::identity_from_model(&model);
        actuation.actuators.pop();
        actuation.generalized_from_actuator = actuation
            .generalized_from_actuator
            .columns(0, model.dof - 1)
            .into_owned();
        let mut reverse = DMatrix::zeros(model.dof - 1, model.dof);
        for coordinate in 0..model.dof - 1 {
            reverse[(coordinate, coordinate)] = 1.0;
        }
        actuation.actuator_from_generalized = Some(reverse);
        actuation.validate(&model).unwrap();
        let bounds = VelocityBounds {
            lower: DVector::from_element(model.dof - 1, -10.0),
            upper: DVector::from_element(model.dof - 1, 10.0),
        };
        assert!(matches!(
            validate_actuator_effort_input(
                Some(ActuatorEffortInput {
                    actuation: &actuation,
                    bounds: &bounds,
                }),
                &model,
                model.dof,
            ),
            Err(DynamicWbcError::UnsupportedActuationTopology { .. })
        ));
    }

    #[test]
    fn friction_pyramid_clamps_an_infeasible_tangential_force_request() {
        let model = upkie();
        let controller = DynamicWbc::new(model.clone(), DynamicWbcConfig::default()).unwrap();
        let state = RobotState::zeros(&model);
        let desired_acceleration = DVector::zeros(model.dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -100.0),
            upper: DVector::from_element(model.dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let mut contact = ContactSpec::horizontal(
            1,
            model.frame_id("left_wheel_center").unwrap(),
            Vec3::zeros(),
            ContactMode::RollingPoint,
            0.5,
            200.0,
            100.0,
        );
        contact.nominal_tangent_x_force = 200.0;
        let output = controller
            .solve(
                DynamicWbcInput {
                    state: &state,
                    desired_acceleration: &desired_acceleration,
                    acceleration_bounds: &acceleration_bounds,
                    torque_bounds: &torque_bounds,
                    actuator_effort: None,
                    contacts: std::slice::from_ref(&contact),
                },
                1,
            )
            .unwrap();
        assert!(
            matches!(
                output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ),
            "status {:?}, dynamics {}, contact {}, violation {}, active {:?}",
            output.status,
            output.dynamics_residual_linf,
            output.contact_acceleration_residual_linf,
            output.solve.maximum_constraint_violation,
            output.solve.active_constraints
        );
        assert!(output.minimum_friction_margin >= -1e-8);
        assert!(
            output.contact_force_basis[(0, 0)] <= 0.5 * output.contact_force_basis[(2, 0)] + 1e-8
        );
        assert!(
            output.minimum_friction_margin < 1e-6,
            "friction constraint did not bind: margin {}, force {:?}, active {:?}",
            output.minimum_friction_margin,
            output.contact_force_basis.column(0),
            output.solve.active_constraints
        );
    }

    #[test]
    fn rolling_wheel_row_couples_center_and_wheel_and_stabilizes_velocity() {
        let model = upkie();
        let generalized_dof = model.dof + 6;
        let wheel_coordinate = model
            .joint_id("left_wheel")
            .and_then(|joint| model.joints[joint.0].coordinate)
            .unwrap();
        let contact = ContactSpec::horizontal(
            7,
            model.frame_id("left_wheel_center").unwrap(),
            Vec3::zeros(),
            ContactMode::RollingWheel {
                coordinate: wheel_coordinate,
                velocity_coefficient: -0.05,
                velocity_stabilization_gain: 10.0,
                maximum_stabilization_acceleration: 3.0,
            },
            0.8,
            100.0,
            50.0,
        );
        let mut jacobian = DMatrix::zeros(3, generalized_dof);
        jacobian[(0, 3)] = 1.0;
        let mut joint_velocity = vec![0.0; model.dof];
        joint_velocity[wheel_coordinate] = 4.0;
        let mut root_twist = Motion6::default();
        root_twist.0[3] = 0.5;
        let mut constraints = ConstraintBuffer::new(generalized_dof, 3);

        emit_floating_contact_acceleration_rows(
            &mut constraints,
            &jacobian,
            Vec3::new(0.2, 0.0, 0.0),
            contact,
            generalized_dof,
            root_twist,
            &joint_velocity,
        )
        .unwrap();

        let rolling = &constraints.active()[0];
        assert_eq!(constraints.active().len(), 3);
        assert_eq!(rolling.coefficients[3], 1.0);
        assert_eq!(rolling.coefficients[6 + wheel_coordinate], -0.05);
        // v_roll = 0.5 - 0.05 * 4 = 0.3 m/s. The -3 m/s²
        // correction saturates, then the 0.2 m/s² bias is subtracted.
        assert!((rolling.lower + 3.2).abs() < 1e-12);
        assert_eq!(rolling.lower, rolling.upper);

        // The same augmented tangent Jacobian must map contact force into
        // generalized dynamics. Otherwise the constraint can demand wheel
        // acceleration without the equal virtual-work contribution from the
        // ground force, producing a state-local solution no plant can realize.
        let force_base = generalized_dof + model.dof;
        let mut dynamics = ConstraintBuffer::new(force_base + 3, generalized_dof);
        for _ in 0..generalized_dof {
            dynamics.push().unwrap();
        }
        augment_floating_dynamics_with_contact(
            &mut dynamics,
            &jacobian,
            contact,
            model.dof,
            force_base,
            0,
        );
        let wheel_equation = &dynamics.active()[6 + wheel_coordinate];
        assert_eq!(wheel_equation.coefficients[force_base], 0.05);
        let root_x_equation = &dynamics.active()[3];
        assert_eq!(root_x_equation.coefficients[force_base], -1.0);
    }

    #[test]
    fn floating_base_solve_supports_upkie_without_a_root_actuator() {
        let model = upkie();
        let controller =
            FloatingDynamicWbc::new(model.clone(), DynamicWbcConfig::default()).unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let desired_acceleration = DVector::zeros(generalized_dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let total_weight: f64 = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
        let contacts = [
            ContactSpec::horizontal(
                1,
                model.frame_id("left_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
            ContactSpec::horizontal(
                2,
                model.frame_id("right_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
        ];
        let input = FloatingDynamicWbcInput {
            state: &state,
            root_twist_world: Motion6::default(),
            desired_generalized_acceleration: &desired_acceleration,
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
        let mut scratch = FloatingDynamicWbcScratch::new(&model, 2);
        let mut output = FloatingDynamicWbcOutput::workspace(model.dof, 2, generalized_dof + 2 * 8);
        controller
            .solve_into(input, &mut output, &mut scratch)
            .unwrap();
        if output.status == SolveStatus::PrimalInfeasible {
            let equalities = scratch
                .constraints
                .active()
                .iter()
                .filter(|row| row.lower == row.upper)
                .collect::<Vec<_>>();
            let variables = generalized_dof + model.dof + 6;
            let matrix = DMatrix::from_fn(equalities.len(), variables, |row, column| {
                equalities[row].coefficients[column]
            });
            let target =
                DVector::from_iterator(equalities.len(), equalities.iter().map(|row| row.lower));
            let equality_solution = matrix
                .clone()
                .svd(true, true)
                .solve(&target, 1e-12)
                .unwrap();
            let residual = (&matrix * &equality_solution - target).amax();
            panic!(
                "feasibility rejected equalities with least-squares residual {residual}, \
                 left force {:?}, right force {:?}",
                &equality_solution.as_slice()
                    [generalized_dof + model.dof..generalized_dof + model.dof + 3],
                &equality_solution.as_slice()
                    [generalized_dof + model.dof + 3..generalized_dof + model.dof + 6],
            );
        }
        assert!(
            matches!(
                output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ),
            "status {:?}, dynamics {}, contact {}, violation {}, active {:?}",
            output.status,
            output.dynamics_residual_linf,
            output.contact_acceleration_residual_linf,
            output.solve.maximum_constraint_violation,
            output.solve.active_constraints
        );
        assert!(
            output.dynamics_residual_linf < 1e-8,
            "dynamics residual {}, status {:?}, violation {}",
            output.dynamics_residual_linf,
            output.status,
            output.solve.maximum_constraint_violation
        );
        assert!(output.contact_acceleration_residual_linf < 1e-8);
        assert!(output.minimum_friction_margin >= -1e-8);
        assert!(output.minimum_torque_margin >= -1e-8);
        assert_eq!(
            output.task_residuals.len(),
            FLOATING_TASK_DIAGNOSTIC_CAPACITY
        );
        assert_eq!(output.task_residuals[0].stable_id, 1);
        assert_eq!(output.task_residuals[0].kind, TaskKind::Orientation);
        assert_eq!(output.task_residuals[0].rows, 3);
        assert!(output.task_residuals[0].active);
        assert!(output.task_residuals[0].rms.is_finite());
        assert_eq!(output.task_residuals[4].stable_id, 5);
        assert!(!output.task_residuals[4].active);
        assert_eq!(output.task_residuals[5].stable_id, 6);
        assert!(!output.task_residuals[5].active);
        assert_eq!(output.task_residuals[6].stable_id, 7);
        assert_eq!(output.task_residuals[6].kind, TaskKind::CentroidalMomentum);
        assert!(!output.task_residuals[6].active);
        for task in output.task_residuals.iter().filter(|task| task.active) {
            assert_eq!(
                task.clipped,
                output.solve.clipped_levels.contains(&task.priority)
            );
        }
        // Upkie's authored neutral pose is not statically balanced over its
        // wheel axle, so a correct rolling-contact solution has a small pitch
        // and fore-aft balancing acceleration rather than a fictitious root
        // actuator holding q̈ = 0.
        assert!(output.generalized_acceleration[4].abs() < 1e-8);
        assert!(output.generalized_acceleration[5].abs() < 1e-8);
        assert!(output.generalized_acceleration[1].abs() > 1e-3);
        assert!(output.generalized_acceleration[3].abs() > 1e-3);
        let supported_weight =
            output.contact_force_basis[(2, 0)] + output.contact_force_basis[(2, 1)];
        assert!((supported_weight - total_weight).abs() < 1e-2);

        let mut model_cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut model_cache).unwrap();
        let mut dynamics_cache = DynamicsCache::new(&model);
        let mut com_jacobian = DMatrix::zeros(3, generalized_dof);
        model
            .floating_com_jacobian_into(&model_cache, &mut dynamics_cache, &mut com_jacobian)
            .unwrap();
        let baseline_com_acceleration = &com_jacobian * &output.generalized_acceleration;
        let com_target = Vec3::new(
            baseline_com_acceleration[0],
            baseline_com_acceleration[1],
            baseline_com_acceleration[2],
        );
        let mut centroidal_output =
            FloatingDynamicWbcOutput::workspace(model.dof, 2, generalized_dof + 2 * 8);
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    center_of_mass_task: Some(FloatingCenterOfMassTask {
                        desired_acceleration_world: com_target,
                        horizontal_only: false,
                        priority: Priority::Style,
                        weight: 1.0,
                    }),
                    centroidal_angular_momentum_task: None,
                    ..input
                },
                &mut centroidal_output,
                &mut scratch,
            )
            .unwrap();
        assert!(matches!(
            centroidal_output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ));
        let achieved_com_acceleration = &com_jacobian * &centroidal_output.generalized_acceleration;
        assert!(
            (&achieved_com_acceleration - &baseline_com_acceleration).norm() < 1e-8,
            "CoM acceleration row was not honored: target {com_target:?}, achieved \
             {achieved_com_acceleration:?}"
        );

        let contact_moment_about_com = |result: &FloatingDynamicWbcOutput| {
            contacts
                .iter()
                .enumerate()
                .fold(Vec3::zeros(), |moment, (slot, contact)| {
                    let point_world = model_cache.world_from_body[contact.frame.0]
                        .transform_point(&Point3::from(contact.point_in_frame))
                        .coords;
                    let force_world = contact.tangent_x_world
                        * result.contact_force_basis[(0, slot)]
                        + contact.tangent_y_world * result.contact_force_basis[(1, slot)]
                        + contact.normal_world * result.contact_force_basis[(2, slot)];
                    moment + (point_world - model_cache.center_of_mass_world).cross(&force_world)
                })
        };
        let baseline_angular_momentum_rate = contact_moment_about_com(&output);
        let mut momentum_output =
            FloatingDynamicWbcOutput::workspace(model.dof, 2, generalized_dof + 2 * 8);
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    centroidal_angular_momentum_task: Some(FloatingCentroidalAngularMomentumTask {
                        desired_rate_world: baseline_angular_momentum_rate,
                        priority: Priority::Style,
                        weight: 1.0,
                    }),
                    ..input
                },
                &mut momentum_output,
                &mut scratch,
            )
            .unwrap();
        assert!(matches!(
            momentum_output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ));
        let achieved_angular_momentum_rate = contact_moment_about_com(&momentum_output);
        assert!(
            (achieved_angular_momentum_rate - baseline_angular_momentum_rate).norm() < 1e-8,
            "centroidal momentum-rate row was not honored: target \
             {baseline_angular_momentum_rate:?}, achieved {achieved_angular_momentum_rate:?}"
        );
        assert!(momentum_output.task_residuals[6].active);
        assert_eq!(
            momentum_output.task_residuals[6].kind,
            TaskKind::CentroidalMomentum
        );
        assert!(momentum_output.task_residuals[6].rms < 1e-8);
    }

    #[test]
    fn fixed_effort_query_uses_compact_equality_layout_and_reconstructs_solution() {
        let model = upkie();
        let controller =
            FloatingDynamicWbc::new(model.clone(), DynamicWbcConfig::default()).unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let desired_acceleration = DVector::zeros(generalized_dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let total_weight: f64 = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
        let contacts = [
            ContactSpec::horizontal(
                1,
                model.frame_id("left_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
            ContactSpec::horizontal(
                2,
                model.frame_id("right_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
        ];
        let input = FloatingDynamicWbcInput {
            state: &state,
            root_twist_world: Motion6::default(),
            desired_generalized_acceleration: &desired_acceleration,
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
        let ordinary = controller.solve(input, 2).unwrap();
        assert!(matches!(
            ordinary.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ));
        let mut compact_scratch = FloatingDynamicWbcScratch::new_fixed_effort(&model, 2);
        let mut realized =
            FloatingDynamicWbcOutput::workspace(model.dof, 2, generalized_dof + 2 * 8);
        controller
            .solve_with_fixed_generalized_effort_into(
                input,
                &ordinary.actuator_torque,
                &mut realized,
                &mut compact_scratch,
            )
            .unwrap();
        assert_eq!(realized.status, SolveStatus::Solved);
        assert_eq!(realized.actuator_torque, ordinary.actuator_torque);
        assert!(realized.dynamics_residual_linf < 1e-8);
        assert!(realized.contact_acceleration_residual_linf < 1e-8);
        assert!(realized.solve.maximum_constraint_violation < 1e-8);
        assert!(
            (&realized.generalized_acceleration - &ordinary.generalized_acceleration).amax() < 1e-7
        );
    }

    #[test]
    fn floating_point_task_reports_physical_acceleration_residual() {
        let model = toy_humanoid();
        let controller = FloatingDynamicWbc::new(
            model.clone(),
            DynamicWbcConfig {
                gravity_world: Vec3::zeros(),
                ..DynamicWbcConfig::default()
            },
        )
        .unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let desired_acceleration = DVector::zeros(generalized_dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let desired_point_acceleration = Vec3::new(0.4, -0.2, 0.1);
        let point_tasks = [FloatingPointAccelerationTask {
            stable_id: 42,
            frame: model.frame_id("left_hand").unwrap(),
            point_in_frame: Vec3::zeros(),
            desired_acceleration_world: desired_point_acceleration,
            priority: Priority::Intent,
            weight: 1.0,
        }];
        let mut scratch = FloatingDynamicWbcScratch::new(&model, 0);
        let mut output = FloatingDynamicWbcOutput::workspace(model.dof, 0, generalized_dof);
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &state,
                    root_twist_world: Motion6::default(),
                    desired_generalized_acceleration: &desired_acceleration,
                    task_priorities: FloatingTaskPriorities::default(),
                    task_weights: FloatingTaskWeights {
                        root_angular: 0.0,
                        root_horizontal: 0.0,
                        root_height: 0.0,
                        joint_posture: 0.0,
                    },
                    joint_posture_weight: 0.0,
                    joint_acceleration_task: None,
                    center_of_mass_task: None,
                    centroidal_angular_momentum_task: None,
                    frame_angular_acceleration_tasks: &[],
                    point_acceleration_tasks: &point_tasks,
                    generalized_acceleration_bounds: &acceleration_bounds,
                    torque_bounds: &torque_bounds,
                    actuator_effort: None,
                    contacts: &[],
                    support_patches: &[],
                },
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert!(
            matches!(
                output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ),
            "status {:?}, solve {:?}",
            output.status,
            output.solve
        );
        let bias = model
            .point_bias_acceleration_world(
                point_tasks[0].frame,
                point_tasks[0].point_in_frame,
                &scratch.model,
                &scratch.dynamics,
            )
            .unwrap();
        let achieved = &scratch.point_task_jacobian * &output.generalized_acceleration + bias;
        let physical_residual = (achieved - desired_point_acceleration).norm();
        let diagnostic = output.task_residuals[9 + FLOATING_ANGULAR_TASK_CAPACITY];
        assert_eq!(diagnostic.stable_id, 42);
        assert_eq!(diagnostic.kind, TaskKind::Point);
        assert_eq!(diagnostic.rows, 3);
        assert!(diagnostic.active);
        assert!((diagnostic.l2 - physical_residual).abs() < 1e-12);
        assert!(physical_residual < 1e-8);
        for task in &output.task_residuals[10 + FLOATING_ANGULAR_TASK_CAPACITY..] {
            assert!(!task.active);
        }
    }

    #[test]
    fn floating_frame_angular_task_reports_physical_residual() {
        let model = toy_humanoid();
        let controller = FloatingDynamicWbc::new(
            model.clone(),
            DynamicWbcConfig {
                gravity_world: Vec3::zeros(),
                ..DynamicWbcConfig::default()
            },
        )
        .unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let desired_acceleration = DVector::zeros(generalized_dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let desired = Vec3::new(0.3, -0.15, 0.2);
        let angular_tasks = [FloatingFrameAngularAccelerationTask {
            stable_id: 42,
            frame: model.frame_id("left_hand").unwrap(),
            desired_angular_acceleration_world: desired,
            priority: Priority::Intent,
            weight: 1.0,
        }];
        let mut scratch = FloatingDynamicWbcScratch::new(&model, 0);
        let mut output = FloatingDynamicWbcOutput::workspace(model.dof, 0, generalized_dof);
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &state,
                    root_twist_world: Motion6::default(),
                    desired_generalized_acceleration: &desired_acceleration,
                    task_priorities: FloatingTaskPriorities::default(),
                    task_weights: FloatingTaskWeights {
                        root_angular: 0.0,
                        root_horizontal: 0.0,
                        root_height: 0.0,
                        joint_posture: 0.0,
                    },
                    joint_posture_weight: 0.0,
                    joint_acceleration_task: None,
                    center_of_mass_task: None,
                    centroidal_angular_momentum_task: None,
                    frame_angular_acceleration_tasks: &angular_tasks,
                    point_acceleration_tasks: &[],
                    generalized_acceleration_bounds: &acceleration_bounds,
                    torque_bounds: &torque_bounds,
                    actuator_effort: None,
                    contacts: &[],
                    support_patches: &[],
                },
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert!(
            matches!(
                output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ),
            "status {:?}, solve {:?}",
            output.status,
            output.solve
        );
        let bias = model
            .angular_bias_acceleration_world(angular_tasks[0].frame, &scratch.dynamics)
            .unwrap();
        let achieved = &scratch.angular_task_jacobian * &output.generalized_acceleration + bias;
        let physical_residual = (achieved - desired).norm();
        let diagnostic = output.task_residuals[9];
        assert_eq!(diagnostic.stable_id, 42);
        assert_eq!(diagnostic.kind, TaskKind::Orientation);
        assert_eq!(diagnostic.rows, 3);
        assert!(diagnostic.active);
        assert!((diagnostic.l2 - physical_residual).abs() < 1e-12);
        assert!(physical_residual < 1e-8);
    }

    #[test]
    fn contact_force_style_does_not_drive_static_posture_acceleration() {
        let model = toy_humanoid();
        let controller = FloatingDynamicWbc::new(
            model.clone(),
            DynamicWbcConfig {
                gravity_world: Vec3::zeros(),
                ..DynamicWbcConfig::default()
            },
        )
        .unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let desired_acceleration = DVector::zeros(generalized_dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -1_000.0),
            upper: DVector::from_element(model.dof, 1_000.0),
        };
        let foot = model.frame_id("left_foot").unwrap();
        let contacts = [
            ContactSpec::horizontal(
                1,
                foot,
                Vec3::new(-0.03, 0.0, -0.07),
                ContactMode::LockedPoint,
                1.0,
                500.0,
                100.0,
            ),
            ContactSpec::horizontal(
                2,
                foot,
                Vec3::new(0.19, 0.0, -0.07),
                ContactMode::LockedPoint,
                1.0,
                500.0,
                100.0,
            ),
        ];
        let mut scratch = FloatingDynamicWbcScratch::new(&model, contacts.len());
        let mut output = FloatingDynamicWbcOutput::workspace(
            model.dof,
            contacts.len(),
            generalized_dof + contacts.len() * 8,
        );
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &state,
                    root_twist_world: Motion6::default(),
                    desired_generalized_acceleration: &desired_acceleration,
                    task_priorities: FloatingTaskPriorities {
                        joint_posture: Priority::Preference,
                        ..FloatingTaskPriorities::default()
                    },
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
                },
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert!(
            matches!(
                output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ),
            "status {:?}, solve {:?}",
            output.status,
            output.solve
        );
        assert!(
            output.generalized_acceleration.amax() < 1e-8,
            "style-level nominal-force regularization changed a fully specified \
             zero-acceleration posture: {:?}",
            output.generalized_acceleration
        );
        assert!(output.task_residuals[3].rms < 1e-8);
        assert!(output.task_residuals[7].rms > 1.0);
    }

    #[test]
    fn floating_double_support_preserves_root_invariants_over_viability_com() {
        let model = toy_humanoid();
        let controller =
            FloatingDynamicWbc::new(model.clone(), DynamicWbcConfig::default()).unwrap();
        let mut state = RobotState::zeros(&model);
        for (name, value) in [
            ("left_hip_pitch", -0.15),
            ("right_hip_pitch", -0.15),
            ("left_knee", 0.3),
            ("right_knee", 0.3),
            ("left_ankle", -0.15),
            ("right_ankle", -0.15),
        ] {
            let joint = model.joint_id(name).unwrap();
            state.q[model.joints[joint.0].coordinate.unwrap()] = value;
        }
        let generalized_dof = model.dof + 6;
        let mut desired_acceleration = DVector::zeros(generalized_dof);
        desired_acceleration[0] = 0.2;
        desired_acceleration[1] = -0.1;
        desired_acceleration[2] = 0.05;
        desired_acceleration[5] = 1.0;
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -200.0),
            upper: DVector::from_element(generalized_dof, 200.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(model.dof, -2_000.0),
            upper: DVector::from_element(model.dof, 2_000.0),
        };
        let total_weight: f64 = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
        let contacts = [
            ContactSpec::horizontal(
                1,
                model.frame_id("left_foot").unwrap(),
                Vec3::zeros(),
                ContactMode::LockedPoint,
                0.8,
                2.0 * total_weight,
                0.5 * total_weight,
            ),
            ContactSpec::horizontal(
                2,
                model.frame_id("right_foot").unwrap(),
                Vec3::zeros(),
                ContactMode::LockedPoint,
                0.8,
                2.0 * total_weight,
                0.5 * total_weight,
            ),
        ];
        let mut scratch = FloatingDynamicWbcScratch::new(&model, 2);
        let mut output = FloatingDynamicWbcOutput::workspace(model.dof, 2, generalized_dof + 16);
        controller
            .solve_into(
                FloatingDynamicWbcInput {
                    state: &state,
                    root_twist_world: Motion6::default(),
                    desired_generalized_acceleration: &desired_acceleration,
                    task_priorities: FloatingTaskPriorities {
                        root_angular: Priority::Invariant,
                        root_horizontal: Priority::Intent,
                        root_height: Priority::Invariant,
                        joint_posture: Priority::Preference,
                    },
                    task_weights: FloatingTaskWeights {
                        root_angular: 1.0,
                        root_horizontal: 0.0,
                        root_height: 1.0,
                        joint_posture: 0.0,
                    },
                    joint_posture_weight: 0.0,
                    joint_acceleration_task: None,
                    center_of_mass_task: Some(FloatingCenterOfMassTask {
                        desired_acceleration_world: Vec3::new(10.0, -10.0, 0.0),
                        horizontal_only: true,
                        priority: Priority::Viability,
                        weight: 1.0,
                    }),
                    centroidal_angular_momentum_task: None,
                    frame_angular_acceleration_tasks: &[],
                    point_acceleration_tasks: &[],
                    generalized_acceleration_bounds: &acceleration_bounds,
                    torque_bounds: &torque_bounds,
                    actuator_effort: None,
                    contacts: &contacts,
                    support_patches: &[],
                },
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert!(
            matches!(
                output.status,
                SolveStatus::Solved | SolveStatus::SolvedWithSlack
            ),
            "status {:?}, solve {:?}",
            output.status,
            output.solve
        );
        for axis in 0..3 {
            assert!(
                (output.generalized_acceleration[axis] - desired_acceleration[axis]).abs() < 1e-8,
                "root angular axis {axis} changed from {} to {}",
                desired_acceleration[axis],
                output.generalized_acceleration[axis],
            );
        }
        assert!(
            (output.generalized_acceleration[5] - 1.0).abs() < 1e-8,
            "root z acceleration {}, task residual {}, dynamics {}, contact {}, forces {:?}, \
             friction {}, clipped {:?}, active {:?}",
            output.generalized_acceleration[5],
            output.task_residuals[2].l2,
            output.dynamics_residual_linf,
            output.contact_acceleration_residual_linf,
            output.contact_force_basis,
            output.minimum_friction_margin,
            output.solve.clipped_levels,
            output.solve.active_constraints,
        );
    }
}
