use anyhow::{Context, Result};
use bonesaw_core::{
    CompiledModel, CompiledSignalProgram, CompiledTaskProgram, ContactMode, ModelCache, Priority,
    RobotState, SignalOp, SignalOutputSpec, TaskSpec, Vec3,
};

mod upkie_landing;
pub use upkie_landing::{
    UPKIE_MEASURED_LANDING_DIAGNOSTIC_WIDTH, UpkieMeasuredLandingConfig,
    UpkieMeasuredLandingOutput, UpkieMeasuredLandingState,
    apply_upkie_measured_landing_request_envelope, step_upkie_measured_landing,
};
mod upkie_moment;
pub use upkie_moment::{
    UPKIE_BODY_MOMENT_DIAGNOSTIC_WIDTH, UpkieBodyMomentRejectionConfig,
    UpkieBodyMomentRejectionOutput, step_upkie_body_moment_rejection,
};

#[derive(Clone, Copy, Debug)]
pub struct UpkieWheelBalancerConfig {
    pub wheel_radius: f64,
    pub position_damping: f64,
    pub position_stiffness: f64,
    pub pitch_damping: f64,
    pub pitch_stiffness: f64,
    pub maximum_ground_velocity: f64,
    pub maximum_integral_velocity: f64,
    pub wheel_velocity_bandwidth: f64,
    pub maximum_wheel_acceleration: f64,
}

impl Default for UpkieWheelBalancerConfig {
    fn default() -> Self {
        Self {
            wheel_radius: 0.05,
            position_damping: 1.1,
            position_stiffness: 2.4,
            pitch_damping: 2.2,
            pitch_stiffness: 24.0,
            maximum_ground_velocity: 2.0,
            maximum_integral_velocity: 10.0,
            wheel_velocity_bandwidth: 40.0,
            maximum_wheel_acceleration: 200.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UpkieWheelBalancerState {
    pub integral_velocity: f64,
}

/// Configuration for the sagittal capture/station-keeping layer used by the
/// Upkie example. This is deliberately robot-example policy, not a generic
/// assumption inside `bonesaw-core`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieCaptureReferenceConfig {
    pub gravity_mps2: f64,
    pub minimum_com_height_m: f64,
    pub maximum_com_height_m: f64,
    /// Fraction of the instantaneous `com_velocity / omega` capture offset
    /// presented to the reference-matched pitch loop. The full DCM remains the
    /// authority witness even when plant bandwidth requires a smaller command.
    pub capture_velocity_fraction: f64,
    pub station_release_capture_error_m: f64,
    pub station_zero_authority_capture_error_m: f64,
}

impl Default for UpkieCaptureReferenceConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: 9.81,
            minimum_com_height_m: 0.05,
            maximum_com_height_m: 1.0,
            capture_velocity_fraction: 0.2,
            station_release_capture_error_m: 0.005,
            station_zero_authority_capture_error_m: 0.03,
        }
    }
}

/// Explicit persistent state for the capture reference. The actuator-facing
/// update stays identical to the reference-matched Upkie PI law; only its
/// capture-angle and authority-scaled station inputs differ.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UpkieCaptureReferenceState {
    pub reference_state: UpkieWheelBalancerState,
    pub commanded_ground_velocity_mps: f64,
}

/// Continuous authority evidence emitted beside the wheel acceleration rows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieCaptureReferenceOutput {
    pub center_of_mass_height_m: f64,
    pub used_center_of_mass_height_m: f64,
    pub natural_frequency_rad_s: f64,
    pub capture_position_control_world_m: f64,
    pub capture_error_m: f64,
    pub capture_pressure: f64,
    pub station_error_m: f64,
    pub station_authority: f64,
    pub unslewed_ground_velocity_mps: f64,
    pub commanded_ground_velocity_mps: f64,
    pub ground_velocity_was_saturated: bool,
    pub ground_acceleration_was_saturated: bool,
}

/// Differential-wheel steering layered around the sagittal capture law.
/// This remains Upkie example policy: the generic WBC only consumes the
/// resulting root/angular and wheel-acceleration tasks plus explicit contact
/// bases.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkiePlanarCaptureConfig {
    pub sagittal: UpkieCaptureReferenceConfig,
    pub steering_release_capture_error_m: f64,
    pub steering_full_capture_error_m: f64,
    pub heading_gain_rad_s_per_rad: f64,
    pub maximum_yaw_rate_rad_s: f64,
    pub yaw_rate_slew_rad_s2: f64,
    pub yaw_rate_tracking_gain_per_s: f64,
    pub maximum_yaw_acceleration_rad_s2: f64,
}

impl Default for UpkiePlanarCaptureConfig {
    fn default() -> Self {
        Self {
            sagittal: UpkieCaptureReferenceConfig::default(),
            steering_release_capture_error_m: 0.0005,
            steering_full_capture_error_m: 0.005,
            heading_gain_rad_s_per_rad: 0.5,
            maximum_yaw_rate_rad_s: 0.5,
            yaw_rate_slew_rad_s2: 5.0,
            yaw_rate_tracking_gain_per_s: 8.0,
            maximum_yaw_acceleration_rad_s2: 20.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UpkiePlanarCaptureState {
    pub sagittal: UpkieCaptureReferenceState,
    pub commanded_yaw_rate_rad_s: f64,
    /// First cross-track capture direction retained through the recovery event
    /// so rocking does not reverse the wheel heading every control tick.
    pub steering_direction: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkiePlanarCaptureOutput {
    pub sagittal: UpkieCaptureReferenceOutput,
    pub heading_world_rad: f64,
    pub longitudinal_capture_error_m: f64,
    pub lateral_capture_error_m: f64,
    pub planar_capture_error_m: f64,
    /// Signed inward DCM margin to the two-wheel support segment.
    pub lateral_support_margin_m: f64,
    pub lateral_capture_pressure: f64,
    pub planar_capture_pressure: f64,
    pub desired_heading_error_rad: f64,
    pub steering_direction: f64,
    pub target_yaw_rate_rad_s: f64,
    pub commanded_yaw_rate_rad_s: f64,
    pub commanded_yaw_acceleration_rad_s2: f64,
    pub wheel_track_m: f64,
    pub steering_was_saturated: bool,
}

/// Pre-loss lateral DCM action for the Upkie example morphology. This policy
/// does not alter contact rows or claim recovery. It selects a bounded support
/// point inside the measured wheel track, converts that point into a lateral
/// acceleration, and coordinates a bank target with the requested inertial
/// acceleration. The ordinary floating WBC remains the execution authority.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieLateralViabilityConfig {
    pub gravity_mps2: f64,
    pub minimum_com_height_m: f64,
    pub maximum_com_height_m: f64,
    pub support_reserve_m: f64,
    pub activation_release_dcm_m: f64,
    pub activation_full_dcm_m: f64,
    pub dcm_decay_rate_per_s: f64,
    pub maximum_lateral_acceleration_m_s2: f64,
    pub lateral_acceleration_slew_m_s3: f64,
    pub maximum_bank_angle_rad: f64,
    pub bank_angle_slew_rad_s: f64,
    pub bank_tracking_stiffness_per_s2: f64,
    pub bank_tracking_damping_per_s: f64,
    pub maximum_roll_acceleration_rad_s2: f64,
}

impl Default for UpkieLateralViabilityConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: 9.81,
            minimum_com_height_m: 0.20,
            maximum_com_height_m: 0.80,
            support_reserve_m: 0.02,
            activation_release_dcm_m: 0.005,
            activation_full_dcm_m: 0.04,
            dcm_decay_rate_per_s: 4.0,
            maximum_lateral_acceleration_m_s2: 8.0,
            lateral_acceleration_slew_m_s3: 80.0,
            maximum_bank_angle_rad: 25.0_f64.to_radians(),
            bank_angle_slew_rad_s: 3.0,
            bank_tracking_stiffness_per_s2: 80.0,
            bank_tracking_damping_per_s: 14.0,
            maximum_roll_acceleration_rad_s2: 80.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UpkieLateralViabilityState {
    pub commanded_lateral_acceleration_m_s2: f64,
    pub commanded_bank_angle_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieLateralViabilityOutput {
    pub used_com_height_m: f64,
    pub natural_frequency_rad_s: f64,
    pub lateral_dcm_m: f64,
    pub support_limit_m: f64,
    pub viability_margin_m: f64,
    pub unconstrained_zmp_m: f64,
    pub commanded_zmp_m: f64,
    pub requested_lateral_acceleration_m_s2: f64,
    pub commanded_lateral_acceleration_m_s2: f64,
    pub target_bank_angle_rad: f64,
    pub commanded_bank_angle_rad: f64,
    pub commanded_roll_acceleration_rad_s2: f64,
    pub activation_pressure: f64,
    pub zmp_was_saturated: bool,
    pub acceleration_was_saturated: bool,
    pub bank_was_saturated: bool,
}

/// Advance the explicit lateral viability action once. Inputs are measured
/// state in the current wheel-heading frame. Invalid input is atomic.
#[allow(clippy::too_many_arguments)]
pub fn step_upkie_lateral_viability(
    timestep_seconds: f64,
    lateral_com_m: f64,
    lateral_com_velocity_m_s: f64,
    center_of_mass_height_m: f64,
    half_wheel_track_m: f64,
    measured_roll_rad: f64,
    measured_roll_rate_rad_s: f64,
    config: UpkieLateralViabilityConfig,
    state: &mut UpkieLateralViabilityState,
) -> Option<UpkieLateralViabilityOutput> {
    let values = [
        timestep_seconds,
        lateral_com_m,
        lateral_com_velocity_m_s,
        center_of_mass_height_m,
        half_wheel_track_m,
        measured_roll_rad,
        measured_roll_rate_rad_s,
        config.gravity_mps2,
        config.minimum_com_height_m,
        config.maximum_com_height_m,
        config.support_reserve_m,
        config.activation_release_dcm_m,
        config.activation_full_dcm_m,
        config.dcm_decay_rate_per_s,
        config.maximum_lateral_acceleration_m_s2,
        config.lateral_acceleration_slew_m_s3,
        config.maximum_bank_angle_rad,
        config.bank_angle_slew_rad_s,
        config.bank_tracking_stiffness_per_s2,
        config.bank_tracking_damping_per_s,
        config.maximum_roll_acceleration_rad_s2,
        state.commanded_lateral_acceleration_m_s2,
        state.commanded_bank_angle_rad,
    ];
    if values.iter().any(|value| !value.is_finite())
        || timestep_seconds <= 0.0
        || config.gravity_mps2 <= 0.0
        || config.minimum_com_height_m <= 0.0
        || config.minimum_com_height_m > config.maximum_com_height_m
        || config.support_reserve_m < 0.0
        || half_wheel_track_m <= config.support_reserve_m
        || config.activation_release_dcm_m < 0.0
        || config.activation_release_dcm_m >= config.activation_full_dcm_m
        || config.dcm_decay_rate_per_s <= 0.0
        || config.maximum_lateral_acceleration_m_s2 <= 0.0
        || config.lateral_acceleration_slew_m_s3 <= 0.0
        || config.maximum_bank_angle_rad <= 0.0
        || config.maximum_bank_angle_rad >= std::f64::consts::FRAC_PI_2
        || config.bank_angle_slew_rad_s <= 0.0
        || config.bank_tracking_stiffness_per_s2 <= 0.0
        || config.bank_tracking_damping_per_s <= 0.0
        || config.maximum_roll_acceleration_rad_s2 <= 0.0
    {
        return None;
    }

    let used_height =
        center_of_mass_height_m.clamp(config.minimum_com_height_m, config.maximum_com_height_m);
    let omega = (config.gravity_mps2 / used_height).sqrt();
    let lateral_dcm = lateral_com_m + lateral_com_velocity_m_s / omega;
    let support_limit = half_wheel_track_m - config.support_reserve_m;
    let viability_margin = support_limit - lateral_dcm.abs();
    let unconstrained_zmp = lateral_dcm * (1.0 + config.dcm_decay_rate_per_s / omega);
    let commanded_zmp = unconstrained_zmp.clamp(-support_limit, support_limit);
    let raw_lateral_acceleration = omega * omega * (lateral_com_m - commanded_zmp);
    let requested_lateral_acceleration = raw_lateral_acceleration.clamp(
        -config.maximum_lateral_acceleration_m_s2,
        config.maximum_lateral_acceleration_m_s2,
    );
    let phase = ((lateral_dcm.abs() - config.activation_release_dcm_m)
        / (config.activation_full_dcm_m - config.activation_release_dcm_m))
        .clamp(0.0, 1.0);
    let activation_pressure = phase * phase * (3.0 - 2.0 * phase);

    let mut next = *state;
    let maximum_acceleration_step = config.lateral_acceleration_slew_m_s3 * timestep_seconds;
    next.commanded_lateral_acceleration_m_s2 += (requested_lateral_acceleration
        - next.commanded_lateral_acceleration_m_s2)
        .clamp(-maximum_acceleration_step, maximum_acceleration_step);
    let unsaturated_bank = (-next.commanded_lateral_acceleration_m_s2).atan2(config.gravity_mps2);
    let target_bank = unsaturated_bank.clamp(
        -config.maximum_bank_angle_rad,
        config.maximum_bank_angle_rad,
    );
    let maximum_bank_step = config.bank_angle_slew_rad_s * timestep_seconds;
    next.commanded_bank_angle_rad +=
        (target_bank - next.commanded_bank_angle_rad).clamp(-maximum_bank_step, maximum_bank_step);
    let commanded_roll_acceleration = (config.bank_tracking_stiffness_per_s2
        * (next.commanded_bank_angle_rad - measured_roll_rad)
        - config.bank_tracking_damping_per_s * measured_roll_rate_rad_s)
        .clamp(
            -config.maximum_roll_acceleration_rad_s2,
            config.maximum_roll_acceleration_rad_s2,
        );
    *state = next;
    Some(UpkieLateralViabilityOutput {
        used_com_height_m: used_height,
        natural_frequency_rad_s: omega,
        lateral_dcm_m: lateral_dcm,
        support_limit_m: support_limit,
        viability_margin_m: viability_margin,
        unconstrained_zmp_m: unconstrained_zmp,
        commanded_zmp_m: commanded_zmp,
        requested_lateral_acceleration_m_s2: requested_lateral_acceleration,
        commanded_lateral_acceleration_m_s2: next.commanded_lateral_acceleration_m_s2,
        target_bank_angle_rad: target_bank,
        commanded_bank_angle_rad: next.commanded_bank_angle_rad,
        commanded_roll_acceleration_rad_s2: commanded_roll_acceleration,
        activation_pressure,
        zmp_was_saturated: commanded_zmp != unconstrained_zmp,
        acceleration_was_saturated: requested_lateral_acceleration != raw_lateral_acceleration,
        bank_was_saturated: target_bank != unsaturated_bank,
    })
}

/// Measured wheel-load reserve action for a two-wheel support. This action is
/// deliberately upstream of torque admission: it authors bounded lateral and
/// bank accelerations, while the ordinary floating WBC remains the execution
/// authority. Wheel identity is derived from measured support positions, not
/// from a left/right array convention.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieWheelLoadReserveConfig {
    pub gravity_mps2: f64,
    pub minimum_com_height_m: f64,
    pub maximum_com_height_m: f64,
    pub minimum_total_normal_force_n: f64,
    pub activation_release_load_fraction: f64,
    pub activation_full_load_fraction: f64,
    pub activation_release_dcm_m: f64,
    pub activation_full_dcm_m: f64,
    pub load_filter_time_constant_s: f64,
    pub prediction_lookahead_s: f64,
    pub unloading_rate_release_per_s: f64,
    pub unloading_rate_full_per_s: f64,
    pub support_reserve_m: f64,
    pub dcm_decay_rate_per_s: f64,
    pub authority_attack_per_s: f64,
    pub authority_release_per_s: f64,
    pub maximum_lateral_acceleration_m_s2: f64,
    pub lateral_acceleration_slew_m_s3: f64,
    pub maximum_bank_angle_rad: f64,
    pub bank_angle_slew_rad_s: f64,
    pub bank_tracking_stiffness_per_s2: f64,
    pub bank_tracking_damping_per_s: f64,
    pub maximum_roll_acceleration_rad_s2: f64,
    /// When enabled, callers receive no executable reserve authority unless
    /// the current sample contains exact bilateral contact/load evidence.
    /// The persistent state still releases smoothly, so the diagnostic
    /// witness remains observable across a measured support transition.
    pub require_bilateral_evidence_for_authority: bool,
}

impl Default for UpkieWheelLoadReserveConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: 9.81,
            minimum_com_height_m: 0.20,
            maximum_com_height_m: 0.80,
            minimum_total_normal_force_n: 1.0,
            activation_release_load_fraction: 0.47,
            activation_full_load_fraction: 0.38,
            activation_release_dcm_m: 0.005,
            activation_full_dcm_m: 0.030,
            load_filter_time_constant_s: 0.04,
            prediction_lookahead_s: 0.012,
            unloading_rate_release_per_s: 1.50,
            unloading_rate_full_per_s: 5.00,
            support_reserve_m: 0.02,
            dcm_decay_rate_per_s: 4.0,
            authority_attack_per_s: 25.0,
            authority_release_per_s: 12.0,
            maximum_lateral_acceleration_m_s2: 8.0,
            lateral_acceleration_slew_m_s3: 80.0,
            maximum_bank_angle_rad: 25.0_f64.to_radians(),
            bank_angle_slew_rad_s: 3.0,
            bank_tracking_stiffness_per_s2: 80.0,
            bank_tracking_damping_per_s: 14.0,
            maximum_roll_acceleration_rad_s2: 80.0,
            require_bilateral_evidence_for_authority: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieWheelLoadReserveState {
    pub filtered_load_balance: f64,
    pub previous_weaker_load_fraction: f64,
    pub has_previous_load_fraction: bool,
    pub authority: f64,
    pub commanded_lateral_acceleration_m_s2: f64,
    pub commanded_bank_angle_rad: f64,
}

impl Default for UpkieWheelLoadReserveState {
    fn default() -> Self {
        Self {
            filtered_load_balance: 0.0,
            previous_weaker_load_fraction: 0.5,
            has_previous_load_fraction: false,
            authority: 0.0,
            commanded_lateral_acceleration_m_s2: 0.0,
            commanded_bank_angle_rad: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieWheelLoadReserveOutput {
    pub evidence_available: bool,
    pub active: bool,
    pub weaker_support_index: u8,
    pub total_normal_force_n: f64,
    pub support_load_fractions: [f64; 2],
    pub raw_load_balance: f64,
    pub filtered_load_balance: f64,
    pub weaker_load_fraction: f64,
    pub weaker_load_fraction_rate_per_s: f64,
    pub predicted_weaker_load_fraction: f64,
    pub support_center_m: f64,
    pub half_support_track_m: f64,
    pub measured_cop_m: f64,
    pub lateral_dcm_m: f64,
    pub baseline_zmp_m: f64,
    pub restoring_zmp_m: f64,
    pub commanded_zmp_m: f64,
    pub requested_lateral_acceleration_m_s2: f64,
    pub commanded_lateral_acceleration_m_s2: f64,
    pub target_bank_angle_rad: f64,
    pub commanded_bank_angle_rad: f64,
    pub commanded_roll_acceleration_rad_s2: f64,
    pub activation_pressure: f64,
    pub authority: f64,
    pub zmp_was_saturated: bool,
    pub acceleration_was_saturated: bool,
    pub bank_was_saturated: bool,
}

fn smoothstep_unit(value: f64) -> f64 {
    let phase = value.clamp(0.0, 1.0);
    phase * phase * (3.0 - 2.0 * phase)
}

/// Advance the measured wheel-load reserve action once. `support_lateral_m`
/// and the CoM state share one wheel-heading lateral coordinate. Missing or
/// non-bilateral evidence releases authority smoothly; non-finite or invalid
/// input is atomic and returns `None`.
#[allow(clippy::too_many_arguments)]
pub fn step_upkie_wheel_load_reserve(
    timestep_seconds: f64,
    bilateral_contact_evidence: bool,
    support_lateral_m: [f64; 2],
    support_normal_force_n: [f64; 2],
    lateral_com_m: f64,
    lateral_com_velocity_m_s: f64,
    center_of_mass_height_m: f64,
    measured_roll_rad: f64,
    measured_roll_rate_rad_s: f64,
    config: UpkieWheelLoadReserveConfig,
    state: &mut UpkieWheelLoadReserveState,
) -> Option<UpkieWheelLoadReserveOutput> {
    let values = [
        timestep_seconds,
        support_lateral_m[0],
        support_lateral_m[1],
        support_normal_force_n[0],
        support_normal_force_n[1],
        lateral_com_m,
        lateral_com_velocity_m_s,
        center_of_mass_height_m,
        measured_roll_rad,
        measured_roll_rate_rad_s,
        config.gravity_mps2,
        config.minimum_com_height_m,
        config.maximum_com_height_m,
        config.minimum_total_normal_force_n,
        config.activation_release_load_fraction,
        config.activation_full_load_fraction,
        config.activation_release_dcm_m,
        config.activation_full_dcm_m,
        config.load_filter_time_constant_s,
        config.prediction_lookahead_s,
        config.unloading_rate_release_per_s,
        config.unloading_rate_full_per_s,
        config.support_reserve_m,
        config.dcm_decay_rate_per_s,
        config.authority_attack_per_s,
        config.authority_release_per_s,
        config.maximum_lateral_acceleration_m_s2,
        config.lateral_acceleration_slew_m_s3,
        config.maximum_bank_angle_rad,
        config.bank_angle_slew_rad_s,
        config.bank_tracking_stiffness_per_s2,
        config.bank_tracking_damping_per_s,
        config.maximum_roll_acceleration_rad_s2,
        state.filtered_load_balance,
        state.previous_weaker_load_fraction,
        state.authority,
        state.commanded_lateral_acceleration_m_s2,
        state.commanded_bank_angle_rad,
    ];
    let track = (support_lateral_m[1] - support_lateral_m[0]).abs();
    if values.iter().any(|value| !value.is_finite())
        || timestep_seconds <= 0.0
        || support_normal_force_n.iter().any(|force| *force < 0.0)
        || config.gravity_mps2 <= 0.0
        || config.minimum_com_height_m <= 0.0
        || config.minimum_com_height_m > config.maximum_com_height_m
        || config.minimum_total_normal_force_n <= 0.0
        || !(0.0..0.5).contains(&config.activation_full_load_fraction)
        || config.activation_full_load_fraction >= config.activation_release_load_fraction
        || config.activation_release_load_fraction > 0.5
        || config.activation_release_dcm_m < 0.0
        || config.activation_release_dcm_m >= config.activation_full_dcm_m
        || config.load_filter_time_constant_s <= 0.0
        || config.prediction_lookahead_s < 0.0
        || config.unloading_rate_release_per_s < 0.0
        || config.unloading_rate_release_per_s >= config.unloading_rate_full_per_s
        || config.support_reserve_m < 0.0
        || config.dcm_decay_rate_per_s <= 0.0
        || config.authority_attack_per_s <= 0.0
        || config.authority_release_per_s <= 0.0
        || config.maximum_lateral_acceleration_m_s2 <= 0.0
        || config.lateral_acceleration_slew_m_s3 <= 0.0
        || config.maximum_bank_angle_rad <= 0.0
        || config.maximum_bank_angle_rad >= std::f64::consts::FRAC_PI_2
        || config.bank_angle_slew_rad_s <= 0.0
        || config.bank_tracking_stiffness_per_s2 <= 0.0
        || config.bank_tracking_damping_per_s <= 0.0
        || config.maximum_roll_acceleration_rad_s2 <= 0.0
        || !(0.0..=1.0).contains(&state.authority)
    {
        return None;
    }

    let total_normal_force = support_normal_force_n[0] + support_normal_force_n[1];
    let evidence_available =
        bilateral_contact_evidence && total_normal_force >= config.minimum_total_normal_force_n;
    let support_center = 0.5 * (support_lateral_m[0] + support_lateral_m[1]);
    let half_track = 0.5 * track;
    let used_height =
        center_of_mass_height_m.clamp(config.minimum_com_height_m, config.maximum_com_height_m);
    let omega = (config.gravity_mps2 / used_height).sqrt();
    let centered_com = lateral_com_m - support_center;
    let lateral_dcm = centered_com + lateral_com_velocity_m_s / omega;
    // Dynamic projection can make the apparent lateral track smaller than a
    // configured reserve during a fall. Collapse the usable interval to zero
    // instead of turning a recoverable geometry state into a worker error.
    let support_limit = (half_track - config.support_reserve_m).max(0.0);
    let unconstrained_baseline_zmp = lateral_dcm * (1.0 + config.dcm_decay_rate_per_s / omega);
    let baseline_zmp = unconstrained_baseline_zmp.clamp(-support_limit, support_limit);

    let mut next = *state;
    let mut fractions = [0.5; 2];
    let mut weaker_index = 2_u8;
    let mut weaker_fraction = 0.5;
    let mut raw_load_balance = 0.0;
    let mut measured_cop = support_center;
    let mut fraction_rate = 0.0;
    let mut predicted_fraction = 0.5;
    let dcm_pressure = smoothstep_unit(
        (lateral_dcm.abs() - config.activation_release_dcm_m)
            / (config.activation_full_dcm_m - config.activation_release_dcm_m),
    );
    let mut activation_pressure = dcm_pressure;
    let mut restoring_zmp = baseline_zmp;
    if evidence_available {
        fractions = [
            support_normal_force_n[0] / total_normal_force,
            support_normal_force_n[1] / total_normal_force,
        ];
        raw_load_balance = fractions[0] - fractions[1];
        let filter_alpha = 1.0 - (-timestep_seconds / config.load_filter_time_constant_s).exp();
        next.filtered_load_balance +=
            filter_alpha * (raw_load_balance - next.filtered_load_balance);
        weaker_index = if next.filtered_load_balance <= 0.0 {
            0
        } else {
            1
        };
        weaker_fraction = 0.5 * (1.0 - next.filtered_load_balance.abs());
        measured_cop = fractions[0] * support_lateral_m[0] + fractions[1] * support_lateral_m[1];
        if state.has_previous_load_fraction {
            fraction_rate =
                (weaker_fraction - state.previous_weaker_load_fraction) / timestep_seconds;
        }
        predicted_fraction = (weaker_fraction
            + fraction_rate.min(0.0) * config.prediction_lookahead_s)
            .clamp(0.0, 0.5);
        let fraction_pressure = smoothstep_unit(
            (config.activation_release_load_fraction - predicted_fraction)
                / (config.activation_release_load_fraction - config.activation_full_load_fraction),
        );
        let unloading_rate = (-fraction_rate).max(0.0);
        let rate_pressure = smoothstep_unit(
            (unloading_rate - config.unloading_rate_release_per_s)
                / (config.unloading_rate_full_per_s - config.unloading_rate_release_per_s),
        );
        activation_pressure = activation_pressure
            .max(fraction_pressure)
            .max(rate_pressure);
        let weaker_offset = support_lateral_m[weaker_index as usize] - support_center;
        // Load asymmetry is early evidence of lateral momentum. Braking that
        // momentum requires a support point on the loaded side (opposite the
        // weaker wheel), even though this can temporarily spend weak-wheel
        // load reserve. Reacquisition is judged separately by the plant eval.
        if lateral_dcm.abs() <= config.activation_release_dcm_m {
            restoring_zmp = -weaker_offset.signum() * support_limit;
        }
    }

    let target_authority = activation_pressure;
    let authority_rate = if target_authority > next.authority {
        config.authority_attack_per_s
    } else {
        config.authority_release_per_s
    };
    let authority_step = authority_rate * timestep_seconds;
    next.authority += (target_authority - next.authority).clamp(-authority_step, authority_step);
    // The output is a complete candidate action. Its separate `authority`
    // field is the sole blend applied by the caller, avoiding a hidden
    // authority-squared response at partial activation.
    let commanded_zmp = if activation_pressure > 0.0 || next.authority > 0.0 {
        restoring_zmp
    } else {
        baseline_zmp
    };
    let raw_lateral_acceleration = omega * omega * (centered_com - commanded_zmp);
    let requested_lateral_acceleration = raw_lateral_acceleration.clamp(
        -config.maximum_lateral_acceleration_m_s2,
        config.maximum_lateral_acceleration_m_s2,
    );
    let maximum_acceleration_step = config.lateral_acceleration_slew_m_s3 * timestep_seconds;
    next.commanded_lateral_acceleration_m_s2 += (requested_lateral_acceleration
        - next.commanded_lateral_acceleration_m_s2)
        .clamp(-maximum_acceleration_step, maximum_acceleration_step);
    let unsaturated_bank = (-next.commanded_lateral_acceleration_m_s2).atan2(config.gravity_mps2);
    let target_bank = unsaturated_bank.clamp(
        -config.maximum_bank_angle_rad,
        config.maximum_bank_angle_rad,
    );
    let maximum_bank_step = config.bank_angle_slew_rad_s * timestep_seconds;
    next.commanded_bank_angle_rad +=
        (target_bank - next.commanded_bank_angle_rad).clamp(-maximum_bank_step, maximum_bank_step);
    let commanded_roll_acceleration = (config.bank_tracking_stiffness_per_s2
        * (next.commanded_bank_angle_rad - measured_roll_rad)
        - config.bank_tracking_damping_per_s * measured_roll_rate_rad_s)
        .clamp(
            -config.maximum_roll_acceleration_rad_s2,
            config.maximum_roll_acceleration_rad_s2,
        );
    if evidence_available {
        next.previous_weaker_load_fraction = weaker_fraction;
        next.has_previous_load_fraction = true;
    } else {
        next.has_previous_load_fraction = false;
    }
    *state = next;

    let effective_authority =
        if config.require_bilateral_evidence_for_authority && !evidence_available {
            0.0
        } else {
            next.authority
        };
    Some(UpkieWheelLoadReserveOutput {
        evidence_available,
        active: effective_authority > 0.0,
        weaker_support_index: weaker_index,
        total_normal_force_n: total_normal_force,
        support_load_fractions: fractions,
        raw_load_balance,
        filtered_load_balance: next.filtered_load_balance,
        weaker_load_fraction: weaker_fraction,
        weaker_load_fraction_rate_per_s: fraction_rate,
        predicted_weaker_load_fraction: predicted_fraction,
        support_center_m: support_center,
        half_support_track_m: half_track,
        measured_cop_m: measured_cop,
        lateral_dcm_m: lateral_dcm,
        baseline_zmp_m: baseline_zmp,
        restoring_zmp_m: restoring_zmp,
        commanded_zmp_m: commanded_zmp,
        requested_lateral_acceleration_m_s2: requested_lateral_acceleration,
        commanded_lateral_acceleration_m_s2: next.commanded_lateral_acceleration_m_s2,
        target_bank_angle_rad: target_bank,
        commanded_bank_angle_rad: next.commanded_bank_angle_rad,
        commanded_roll_acceleration_rad_s2: commanded_roll_acceleration,
        activation_pressure,
        authority: effective_authority,
        zmp_was_saturated: baseline_zmp != unconstrained_baseline_zmp,
        acceleration_was_saturated: requested_lateral_acceleration != raw_lateral_acceleration,
        bank_was_saturated: target_bank != unsaturated_bank,
    })
}

/// Bounded single-support touchdown request for the Upkie example.
///
/// This is deliberately a request author, not a contact estimator or a
/// torque path.  The caller must supply an exact measured one-wheel support
/// mask and the free wheel's current height/velocity/Jacobian.  The request
/// uses a damped Jacobian-transpose lowering law, is slew/effort bounded, and
/// is intended to be passed through the ordinary floating WBC with the
/// measured support mask.  No geometric touch is treated as authority.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieSingleSupportReacquisitionConfig {
    pub target_wheel_height_m: f64,
    pub vertical_stiffness_per_s2: f64,
    pub vertical_damping_per_s: f64,
    pub maximum_vertical_acceleration_m_s2: f64,
    pub jacobian_damping: f64,
    pub authority_attack_per_s: f64,
    pub authority_release_per_s: f64,
    pub maximum_joint_acceleration_rad_s2: f64,
}

impl Default for UpkieSingleSupportReacquisitionConfig {
    fn default() -> Self {
        Self {
            target_wheel_height_m: 0.05,
            vertical_stiffness_per_s2: 90.0,
            vertical_damping_per_s: 14.0,
            maximum_vertical_acceleration_m_s2: 20.0,
            jacobian_damping: 1.0e-4,
            authority_attack_per_s: 20.0,
            authority_release_per_s: 30.0,
            maximum_joint_acceleration_rad_s2: 120.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UpkieSingleSupportReacquisitionState {
    pub authority: f64,
    pub previous_support_mask: u8,
    pub transition_count: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieSingleSupportReacquisitionOutput {
    pub evidence_available: bool,
    pub active: bool,
    pub support_mask: u8,
    pub lost_support_index: u8,
    pub wheel_height_m: f64,
    pub wheel_velocity_m_s: f64,
    pub target_wheel_height_m: f64,
    pub target_error_m: f64,
    pub requested_vertical_acceleration_m_s2: f64,
    pub commanded_vertical_acceleration_m_s2: f64,
    pub authority: f64,
    pub jacobian_gain: f64,
    pub joint_acceleration_was_saturated: bool,
    pub transition_count: u32,
}

fn valid_single_support_reacquisition_config(
    config: UpkieSingleSupportReacquisitionConfig,
) -> bool {
    config.target_wheel_height_m.is_finite()
        && config.target_wheel_height_m >= 0.0
        && config.vertical_stiffness_per_s2.is_finite()
        && config.vertical_stiffness_per_s2 > 0.0
        && config.vertical_damping_per_s.is_finite()
        && config.vertical_damping_per_s > 0.0
        && config.maximum_vertical_acceleration_m_s2.is_finite()
        && config.maximum_vertical_acceleration_m_s2 > 0.0
        && config.jacobian_damping.is_finite()
        && config.jacobian_damping > 0.0
        && config.authority_attack_per_s.is_finite()
        && config.authority_attack_per_s > 0.0
        && config.authority_release_per_s.is_finite()
        && config.authority_release_per_s > 0.0
        && config.maximum_joint_acceleration_rad_s2.is_finite()
        && config.maximum_joint_acceleration_rad_s2 > 0.0
}

/// Advance one exact measured single-support touchdown request.  The six
/// element arrays are fixed to the Upkie canonical joint order, so this
/// function remains allocation-free and does not perform a runtime name or
/// topology lookup.
pub fn step_upkie_single_support_reacquisition(
    timestep_seconds: f64,
    observation_exact: bool,
    support_mask: u8,
    wheel_height_m: f64,
    wheel_velocity_m_s: f64,
    wheel_joint_jacobian: &[f64; 6],
    joint_velocity: &[f64; 6],
    config: UpkieSingleSupportReacquisitionConfig,
    state: &mut UpkieSingleSupportReacquisitionState,
    joint_acceleration_out: &mut [f64; 6],
) -> Option<UpkieSingleSupportReacquisitionOutput> {
    if !timestep_seconds.is_finite()
        || timestep_seconds <= 0.0
        || !wheel_height_m.is_finite()
        || !wheel_velocity_m_s.is_finite()
        || wheel_joint_jacobian.iter().any(|value| !value.is_finite())
        || joint_velocity.iter().any(|value| !value.is_finite())
        || support_mask > 3
        || !valid_single_support_reacquisition_config(config)
        || !state.authority.is_finite()
        || !(0.0..=1.0).contains(&state.authority)
    {
        return None;
    }

    let active = observation_exact && matches!(support_mask, 1 | 2);
    let lost_support_index = match support_mask {
        1 => 1,
        2 => 0,
        _ => 0,
    };
    if support_mask != state.previous_support_mask {
        state.previous_support_mask = support_mask;
        state.transition_count = state.transition_count.saturating_add(1);
    }
    let target_authority = f64::from(active);
    let maximum_authority_delta = if target_authority > state.authority {
        config.authority_attack_per_s * timestep_seconds
    } else {
        config.authority_release_per_s * timestep_seconds
    };
    state.authority = (state.authority
        + (target_authority - state.authority)
            .clamp(-maximum_authority_delta, maximum_authority_delta))
    .clamp(0.0, 1.0);
    joint_acceleration_out.fill(0.0);

    let target_error = config.target_wheel_height_m - wheel_height_m;
    let raw_vertical_acceleration = config.vertical_stiffness_per_s2 * target_error
        - config.vertical_damping_per_s * wheel_velocity_m_s;
    let commanded_vertical_acceleration = if active {
        raw_vertical_acceleration.clamp(
            -config.maximum_vertical_acceleration_m_s2,
            config.maximum_vertical_acceleration_m_s2,
        ) * state.authority
    } else {
        0.0
    };
    let jacobian_gain = wheel_joint_jacobian
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        + config.jacobian_damping;
    if !jacobian_gain.is_finite() || jacobian_gain <= 0.0 {
        return None;
    }
    let mut maximum_abs_joint_acceleration: f64 = 0.0;
    for coordinate in 0..6 {
        joint_acceleration_out[coordinate] =
            commanded_vertical_acceleration * wheel_joint_jacobian[coordinate] / jacobian_gain;
        maximum_abs_joint_acceleration =
            maximum_abs_joint_acceleration.max(joint_acceleration_out[coordinate].abs());
    }
    let mut joint_acceleration_was_saturated = false;
    if maximum_abs_joint_acceleration > config.maximum_joint_acceleration_rad_s2 {
        let scale = config.maximum_joint_acceleration_rad_s2 / maximum_abs_joint_acceleration;
        for acceleration in joint_acceleration_out.iter_mut() {
            *acceleration *= scale;
        }
        joint_acceleration_was_saturated = true;
    }
    Some(UpkieSingleSupportReacquisitionOutput {
        evidence_available: observation_exact,
        active,
        support_mask,
        lost_support_index,
        wheel_height_m,
        wheel_velocity_m_s,
        target_wheel_height_m: config.target_wheel_height_m,
        target_error_m: target_error,
        requested_vertical_acceleration_m_s2: raw_vertical_acceleration,
        commanded_vertical_acceleration_m_s2: commanded_vertical_acceleration,
        authority: state.authority,
        jacobian_gain,
        joint_acceleration_was_saturated,
        transition_count: state.transition_count,
    })
}

/// Bounded transition from the Upkie example's primary balance objectives to
/// a low-energy contingency. This supervisor does not claim that a fall is
/// preventable. It makes loss of usable command authority explicit, slews the
/// transition, and latches the terminal fallen boundary until an external
/// reset replaces the state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieFallSafeConfig {
    pub tilt_release_rad: f64,
    pub tilt_full_rad: f64,
    pub angular_rate_release_rad_s: f64,
    pub angular_rate_full_rad_s: f64,
    pub height_release_m: f64,
    pub height_full_m: f64,
    pub solver_release_steps: u32,
    pub solver_full_steps: u32,
    pub authority_attack_rate_per_s: f64,
    pub authority_release_rate_per_s: f64,
    pub minimum_contingency_hold_s: f64,
    pub fallen_tilt_rad: f64,
    pub fallen_height_m: f64,
    pub root_angular_damping_per_s: f64,
    pub root_linear_damping_per_s: f64,
    pub joint_damping_per_s: f64,
    pub maximum_root_angular_acceleration_rad_s2: f64,
    pub maximum_root_linear_acceleration_m_s2: f64,
    pub maximum_joint_acceleration_rad_s2: f64,
}

impl Default for UpkieFallSafeConfig {
    fn default() -> Self {
        Self {
            tilt_release_rad: 28.0_f64.to_radians(),
            tilt_full_rad: 40.0_f64.to_radians(),
            angular_rate_release_rad_s: 3.0,
            angular_rate_full_rad_s: 6.0,
            height_release_m: 0.44,
            height_full_m: 0.38,
            // The canonical Upkie plant has a deterministic five-tick
            // numerical startup transient while the soft contacts settle.
            // Treat that measured prefix as the lease, then expose growing
            // stale-command pressure rather than degrading nominal control.
            solver_release_steps: 5,
            solver_full_steps: 12,
            authority_attack_rate_per_s: 8.0,
            authority_release_rate_per_s: 1.0,
            minimum_contingency_hold_s: 0.25,
            fallen_tilt_rad: 45.0_f64.to_radians(),
            fallen_height_m: 0.35,
            root_angular_damping_per_s: 8.0,
            root_linear_damping_per_s: 4.0,
            joint_damping_per_s: 8.0,
            maximum_root_angular_acceleration_rad_s2: 80.0,
            maximum_root_linear_acceleration_m_s2: 40.0,
            maximum_joint_acceleration_rad_s2: 120.0,
        }
    }
}

/// Write the low-energy contingency acceleration authored by the Upkie
/// example profile. This is a damping target, not a claim that contact or
/// effort constraints can realize it; the ordinary WBC admission remains the
/// authority for that question.
pub fn write_upkie_fall_safe_contingency(
    root_twist_world: &[f64],
    joint_velocity: &[f64],
    config: UpkieFallSafeConfig,
    root_angular_acceleration_out: &mut [f64],
    root_linear_acceleration_out: &mut [f64],
    joint_acceleration_out: &mut [f64],
) -> bool {
    let dimensions_valid = root_twist_world.len() == 6
        && root_angular_acceleration_out.len() == 3
        && root_linear_acceleration_out.len() == 3
        && joint_velocity.len() == joint_acceleration_out.len();
    let parameters = [
        config.root_angular_damping_per_s,
        config.root_linear_damping_per_s,
        config.joint_damping_per_s,
        config.maximum_root_angular_acceleration_rad_s2,
        config.maximum_root_linear_acceleration_m_s2,
        config.maximum_joint_acceleration_rad_s2,
    ];
    if !dimensions_valid
        || parameters
            .into_iter()
            .any(|value| !value.is_finite() || value <= 0.0)
        || root_twist_world
            .iter()
            .chain(joint_velocity)
            .any(|value| !value.is_finite())
    {
        return false;
    }
    for axis in 0..3 {
        root_angular_acceleration_out[axis] =
            (-config.root_angular_damping_per_s * root_twist_world[axis]).clamp(
                -config.maximum_root_angular_acceleration_rad_s2,
                config.maximum_root_angular_acceleration_rad_s2,
            );
        root_linear_acceleration_out[axis] =
            (-config.root_linear_damping_per_s * root_twist_world[3 + axis]).clamp(
                -config.maximum_root_linear_acceleration_m_s2,
                config.maximum_root_linear_acceleration_m_s2,
            );
    }
    for (output, velocity) in joint_acceleration_out.iter_mut().zip(joint_velocity) {
        *output = (-config.joint_damping_per_s * velocity).clamp(
            -config.maximum_joint_acceleration_rad_s2,
            config.maximum_joint_acceleration_rad_s2,
        );
    }
    true
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum UpkieFallSafeMode {
    #[default]
    Primary = 0,
    Degraded = 1,
    Contingency = 2,
    Fallen = 3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieFallSafeState {
    pub primary_authority: f64,
    pub hold_remaining_s: f64,
    pub consecutive_nonadmitted_steps: u32,
    pub transition_count: u32,
    pub fallen_latched: bool,
    pub mode: UpkieFallSafeMode,
}

impl Default for UpkieFallSafeState {
    fn default() -> Self {
        Self {
            primary_authority: 1.0,
            hold_remaining_s: 0.0,
            consecutive_nonadmitted_steps: 0,
            transition_count: 0,
            fallen_latched: false,
            mode: UpkieFallSafeMode::Primary,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieFallSafeOutput {
    pub mode: UpkieFallSafeMode,
    pub tilt_pressure: f64,
    pub angular_rate_pressure: f64,
    pub height_pressure: f64,
    pub solver_pressure: f64,
    pub raw_risk: f64,
    pub requested_primary_authority: f64,
    pub primary_authority: f64,
    pub contingency_authority: f64,
    pub fresh_command_authority: f64,
    pub hold_remaining_s: f64,
    pub consecutive_nonadmitted_steps: u32,
    pub transition_count: u32,
    pub limiting_reason_flags: u32,
    pub authority_was_slew_limited: bool,
    pub fallen_latched: bool,
}

pub const UPKIE_FALL_SAFE_TILT: u32 = 1 << 0;
pub const UPKIE_FALL_SAFE_ANGULAR_RATE: u32 = 1 << 1;
pub const UPKIE_FALL_SAFE_HEIGHT: u32 = 1 << 2;
pub const UPKIE_FALL_SAFE_SOLVER: u32 = 1 << 3;
pub const UPKIE_FALL_SAFE_HOLD: u32 = 1 << 4;
pub const UPKIE_FALL_SAFE_FALLEN: u32 = 1 << 5;

fn smooth_pressure(value: f64, release: f64, full: f64) -> f64 {
    let phase = ((value - release) / (full - release)).clamp(0.0, 1.0);
    phase * phase * (3.0 - 2.0 * phase)
}

/// Advance the explicit Upkie contingency state once. Inputs are observation
/// evidence plus the previous WBC admission result; no clock, solver, policy,
/// plant, allocation, or reset is hidden inside this transition.
#[allow(clippy::too_many_arguments)]
pub fn step_upkie_fall_safe(
    timestep_seconds: f64,
    tilt_rad: f64,
    angular_rate_rad_s: f64,
    root_height_m: f64,
    previous_solver_admitted: bool,
    config: UpkieFallSafeConfig,
    state: &mut UpkieFallSafeState,
) -> Option<UpkieFallSafeOutput> {
    let valid = timestep_seconds.is_finite()
        && timestep_seconds > 0.0
        && tilt_rad.is_finite()
        && tilt_rad >= 0.0
        && angular_rate_rad_s.is_finite()
        && angular_rate_rad_s >= 0.0
        && root_height_m.is_finite()
        && config.tilt_release_rad.is_finite()
        && config.tilt_full_rad.is_finite()
        && 0.0 <= config.tilt_release_rad
        && config.tilt_release_rad < config.tilt_full_rad
        && config.tilt_full_rad <= config.fallen_tilt_rad
        && config.angular_rate_release_rad_s.is_finite()
        && config.angular_rate_full_rad_s.is_finite()
        && 0.0 <= config.angular_rate_release_rad_s
        && config.angular_rate_release_rad_s < config.angular_rate_full_rad_s
        && config.height_release_m.is_finite()
        && config.height_full_m.is_finite()
        && config.fallen_height_m.is_finite()
        && config.fallen_height_m <= config.height_full_m
        && config.height_full_m < config.height_release_m
        && config.solver_release_steps < config.solver_full_steps
        && config.authority_attack_rate_per_s.is_finite()
        && config.authority_attack_rate_per_s > 0.0
        && config.authority_release_rate_per_s.is_finite()
        && config.authority_release_rate_per_s > 0.0
        && config.minimum_contingency_hold_s.is_finite()
        && config.minimum_contingency_hold_s >= 0.0
        && config.fallen_tilt_rad.is_finite()
        && config.fallen_tilt_rad > 0.0
        && config.root_angular_damping_per_s.is_finite()
        && config.root_angular_damping_per_s > 0.0
        && config.root_linear_damping_per_s.is_finite()
        && config.root_linear_damping_per_s > 0.0
        && config.joint_damping_per_s.is_finite()
        && config.joint_damping_per_s > 0.0
        && config.maximum_root_angular_acceleration_rad_s2.is_finite()
        && config.maximum_root_angular_acceleration_rad_s2 > 0.0
        && config.maximum_root_linear_acceleration_m_s2.is_finite()
        && config.maximum_root_linear_acceleration_m_s2 > 0.0
        && config.maximum_joint_acceleration_rad_s2.is_finite()
        && config.maximum_joint_acceleration_rad_s2 > 0.0
        && state.primary_authority.is_finite()
        && (0.0..=1.0).contains(&state.primary_authority)
        && state.hold_remaining_s.is_finite()
        && state.hold_remaining_s >= 0.0;
    if !valid {
        return None;
    }

    if previous_solver_admitted {
        state.consecutive_nonadmitted_steps = 0;
    } else {
        state.consecutive_nonadmitted_steps = state.consecutive_nonadmitted_steps.saturating_add(1);
    }
    let tilt_pressure = smooth_pressure(tilt_rad, config.tilt_release_rad, config.tilt_full_rad);
    let angular_rate_pressure = smooth_pressure(
        angular_rate_rad_s,
        config.angular_rate_release_rad_s,
        config.angular_rate_full_rad_s,
    );
    let height_pressure = smooth_pressure(
        config.height_release_m - root_height_m,
        0.0,
        config.height_release_m - config.height_full_m,
    );
    let solver_pressure = smooth_pressure(
        state.consecutive_nonadmitted_steps as f64,
        config.solver_release_steps as f64,
        config.solver_full_steps as f64,
    );
    let raw_risk = tilt_pressure
        .max(angular_rate_pressure)
        .max(height_pressure)
        .max(solver_pressure);
    if tilt_rad >= config.fallen_tilt_rad || root_height_m <= config.fallen_height_m {
        state.fallen_latched = true;
    }
    if raw_risk >= 1.0 {
        state.hold_remaining_s = state
            .hold_remaining_s
            .max(config.minimum_contingency_hold_s);
    } else {
        state.hold_remaining_s = (state.hold_remaining_s - timestep_seconds).max(0.0);
    }
    let requested_primary_authority = if state.fallen_latched || state.hold_remaining_s > 0.0 {
        0.0
    } else {
        1.0 - raw_risk
    };
    let requested_delta = requested_primary_authority - state.primary_authority;
    let maximum_delta = if requested_delta < 0.0 {
        config.authority_attack_rate_per_s * timestep_seconds
    } else {
        config.authority_release_rate_per_s * timestep_seconds
    };
    let applied_delta = requested_delta.clamp(-maximum_delta, maximum_delta);
    state.primary_authority = (state.primary_authority + applied_delta).clamp(0.0, 1.0);
    let mode = if state.fallen_latched {
        UpkieFallSafeMode::Fallen
    } else if state.primary_authority <= 1.0e-6 && (raw_risk >= 1.0 || state.hold_remaining_s > 0.0)
    {
        UpkieFallSafeMode::Contingency
    } else if raw_risk > 0.0 || state.primary_authority < 1.0 {
        UpkieFallSafeMode::Degraded
    } else {
        UpkieFallSafeMode::Primary
    };
    if mode != state.mode {
        state.transition_count = state.transition_count.saturating_add(1);
        state.mode = mode;
    }
    let mut flags = 0_u32;
    if tilt_pressure > 0.0 {
        flags |= UPKIE_FALL_SAFE_TILT;
    }
    if angular_rate_pressure > 0.0 {
        flags |= UPKIE_FALL_SAFE_ANGULAR_RATE;
    }
    if height_pressure > 0.0 {
        flags |= UPKIE_FALL_SAFE_HEIGHT;
    }
    if solver_pressure > 0.0 {
        flags |= UPKIE_FALL_SAFE_SOLVER;
    }
    if state.hold_remaining_s > 0.0 {
        flags |= UPKIE_FALL_SAFE_HOLD;
    }
    if state.fallen_latched {
        flags |= UPKIE_FALL_SAFE_FALLEN;
    }
    Some(UpkieFallSafeOutput {
        mode,
        tilt_pressure,
        angular_rate_pressure,
        height_pressure,
        solver_pressure,
        raw_risk,
        requested_primary_authority,
        primary_authority: state.primary_authority,
        contingency_authority: 1.0 - state.primary_authority,
        fresh_command_authority: 1.0 - solver_pressure,
        hold_remaining_s: state.hold_remaining_s,
        consecutive_nonadmitted_steps: state.consecutive_nonadmitted_steps,
        transition_count: state.transition_count,
        limiting_reason_flags: flags,
        authority_was_slew_limited: applied_delta != requested_delta,
        fallen_latched: state.fallen_latched,
    })
}

#[derive(Clone, Copy, Debug)]
pub struct UpkieWheelBalancer {
    /// Sorted joint-coordinate indexes for deterministic task row order.
    pub coordinates: [usize; 2],
    rolling_velocity_coefficients: [f64; 2],
    left_coordinate: usize,
    right_coordinate: usize,
    left_rolling_velocity_coefficient: f64,
    right_rolling_velocity_coefficient: f64,
    pub config: UpkieWheelBalancerConfig,
}

impl UpkieWheelBalancer {
    /// Compile the two wheel coordinates and reproduce Upkie's reference PI
    /// balance law at the WBC boundary. The reference implementation commands
    /// opposite signed left/right wheel velocities from ground-position and
    /// base-pitch errors; Bonesaw lowers those velocity commands to bounded
    /// wheel acceleration rows.
    pub fn compile(model: &CompiledModel, state: &RobotState) -> Result<Self> {
        let left_joint = model
            .joint_id("left_wheel")
            .context("Upkie left_wheel coordinate is required")?;
        let right_joint = model
            .joint_id("right_wheel")
            .context("Upkie right_wheel coordinate is required")?;
        let left = model.joints[left_joint.0]
            .coordinate
            .context("Upkie left_wheel must be actuated")?;
        let right = model.joints[right_joint.0]
            .coordinate
            .context("Upkie right_wheel must be actuated")?;
        let mut cache = ModelCache::new(model);
        model.forward_kinematics(state, &mut cache)?;
        let rolling_coefficient = |joint_index: usize| {
            let joint = &model.joints[joint_index];
            let world_from_joint_rotation =
                cache.world_from_body[joint.parent.0].rotation * joint.parent_from_joint.rotation;
            let axis_world = world_from_joint_rotation.transform_vector(&joint.axis_in_joint);
            -UpkieWheelBalancerConfig::default().wheel_radius
                * Vec3::x().dot(&axis_world.cross(&Vec3::z()))
        };
        let left_coefficient = rolling_coefficient(left_joint.0);
        let right_coefficient = rolling_coefficient(right_joint.0);
        if left_coefficient.abs() <= 1e-9 || right_coefficient.abs() <= 1e-9 {
            anyhow::bail!("Upkie wheel axes are not transverse to the rolling direction");
        }
        let (coordinates, rolling_velocity_coefficients) = if left < right {
            ([left, right], [left_coefficient, right_coefficient])
        } else {
            ([right, left], [right_coefficient, left_coefficient])
        };
        Ok(Self {
            coordinates,
            rolling_velocity_coefficients,
            left_coordinate: left,
            right_coordinate: right,
            left_rolling_velocity_coefficient: left_coefficient,
            right_rolling_velocity_coefficient: right_coefficient,
            config: UpkieWheelBalancerConfig::default(),
        })
    }

    pub fn left_contact_mode(&self) -> ContactMode {
        ContactMode::RollingWheel {
            coordinate: self.left_coordinate,
            velocity_coefficient: self.left_rolling_velocity_coefficient,
            velocity_stabilization_gain: 2.0,
            maximum_stabilization_acceleration: 1.0,
        }
    }

    pub fn right_contact_mode(&self) -> ContactMode {
        ContactMode::RollingWheel {
            coordinate: self.right_coordinate,
            velocity_coefficient: self.right_rolling_velocity_coefficient,
            velocity_stabilization_gain: 2.0,
            maximum_stabilization_acceleration: 1.0,
        }
    }

    /// Initialize wheel rates that exactly satisfy the two longitudinal
    /// no-slip rows for a common wheel-center ground velocity.
    pub fn set_admissible_ground_velocity(&self, ground_velocity: f64, joint_velocity: &mut [f64]) {
        for row in 0..self.coordinates.len() {
            joint_velocity[self.coordinates[row]] =
                -ground_velocity / self.rolling_velocity_coefficients[row];
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn emit_accelerations(
        &self,
        timestep_seconds: f64,
        target_ground_position: f64,
        ground_position: f64,
        base_pitch: f64,
        joint_velocity: &[f64],
        state: &mut UpkieWheelBalancerState,
        desired_accelerations: &mut [f64; 2],
    ) -> f64 {
        let position_error = target_ground_position - ground_position;
        let pitch_error = -base_pitch;
        state.integral_velocity += (self.config.position_stiffness * position_error
            + self.config.pitch_stiffness * pitch_error)
            * timestep_seconds;
        state.integral_velocity = state.integral_velocity.clamp(
            -self.config.maximum_integral_velocity,
            self.config.maximum_integral_velocity,
        );
        let ground_velocity = (-(self.config.position_damping * position_error
            + self.config.pitch_damping * pitch_error)
            - state.integral_velocity)
            .clamp(
                -self.config.maximum_ground_velocity,
                self.config.maximum_ground_velocity,
            );
        for (row, desired_acceleration) in desired_accelerations.iter_mut().enumerate() {
            let coordinate = self.coordinates[row];
            let target_wheel_velocity = -ground_velocity / self.rolling_velocity_coefficients[row];
            *desired_acceleration = (self.config.wheel_velocity_bandwidth
                * (target_wheel_velocity - joint_velocity[coordinate]))
                .clamp(
                    -self.config.maximum_wheel_acceleration,
                    self.config.maximum_wheel_acceleration,
                );
        }
        ground_velocity
    }

    /// Emit a velocity-dependent capture command with a lower-authority odom
    /// station preference. Capture is the one-dimensional DCM
    /// `com + com_velocity / omega`. The station term fades continuously to
    /// zero as capture error consumes the declared recovery envelope, so a
    /// localization/station preference cannot oppose viability recovery.
    #[allow(clippy::too_many_arguments)]
    pub fn emit_capture_accelerations(
        &self,
        timestep_seconds: f64,
        station_ground_position_control_world_m: f64,
        ground_position_control_world_m: f64,
        ground_velocity_control_world_mps: f64,
        center_of_mass_position_control_world_m: f64,
        center_of_mass_velocity_control_world_mps: f64,
        center_of_mass_height_m: f64,
        joint_velocity: &[f64],
        config: UpkieCaptureReferenceConfig,
        state: &mut UpkieCaptureReferenceState,
        desired_accelerations: &mut [f64; 2],
    ) -> Option<UpkieCaptureReferenceOutput> {
        if !timestep_seconds.is_finite()
            || timestep_seconds <= 0.0
            || !station_ground_position_control_world_m.is_finite()
            || !ground_position_control_world_m.is_finite()
            || !ground_velocity_control_world_mps.is_finite()
            || !center_of_mass_position_control_world_m.is_finite()
            || !center_of_mass_velocity_control_world_mps.is_finite()
            || !center_of_mass_height_m.is_finite()
            || joint_velocity.len() <= self.coordinates[1]
            || joint_velocity.iter().any(|value| !value.is_finite())
            || !config.gravity_mps2.is_finite()
            || config.gravity_mps2 <= 0.0
            || !config.minimum_com_height_m.is_finite()
            || !config.maximum_com_height_m.is_finite()
            || config.minimum_com_height_m <= 0.0
            || config.minimum_com_height_m > config.maximum_com_height_m
            || !config.capture_velocity_fraction.is_finite()
            || !(0.0..=1.0).contains(&config.capture_velocity_fraction)
            || !config.station_release_capture_error_m.is_finite()
            || !config.station_zero_authority_capture_error_m.is_finite()
            || config.station_release_capture_error_m < 0.0
            || config.station_release_capture_error_m
                >= config.station_zero_authority_capture_error_m
            || !state.commanded_ground_velocity_mps.is_finite()
            || !state.reference_state.integral_velocity.is_finite()
        {
            return None;
        }

        let used_height =
            center_of_mass_height_m.clamp(config.minimum_com_height_m, config.maximum_com_height_m);
        let omega = (config.gravity_mps2 / used_height).sqrt();
        let capture_position = center_of_mass_position_control_world_m
            + center_of_mass_velocity_control_world_mps / omega;
        let capture_error = capture_position - ground_position_control_world_m;
        let absolute_capture_error = capture_error.abs();
        let fade = ((absolute_capture_error - config.station_release_capture_error_m)
            / (config.station_zero_authority_capture_error_m
                - config.station_release_capture_error_m))
            .clamp(0.0, 1.0);
        let smooth_fade = fade * fade * (3.0 - 2.0 * fade);
        let station_authority = 1.0 - smooth_fade;
        let station_error =
            station_ground_position_control_world_m - ground_position_control_world_m;
        let effective_station_target =
            ground_position_control_world_m + station_authority * station_error;
        let effective_capture_error = center_of_mass_position_control_world_m
            - ground_position_control_world_m
            + config.capture_velocity_fraction * center_of_mass_velocity_control_world_mps / omega;
        let capture_pitch = effective_capture_error.atan2(used_height);
        let commanded_ground_velocity = self.emit_accelerations(
            timestep_seconds,
            effective_station_target,
            ground_position_control_world_m,
            capture_pitch,
            joint_velocity,
            &mut state.reference_state,
            desired_accelerations,
        );
        state.commanded_ground_velocity_mps = commanded_ground_velocity;

        Some(UpkieCaptureReferenceOutput {
            center_of_mass_height_m,
            used_center_of_mass_height_m: used_height,
            natural_frequency_rad_s: omega,
            capture_position_control_world_m: capture_position,
            capture_error_m: capture_error,
            capture_pressure: (absolute_capture_error
                / config.station_zero_authority_capture_error_m)
                .clamp(0.0, 1.0),
            station_error_m: station_error,
            station_authority,
            unslewed_ground_velocity_mps: commanded_ground_velocity,
            commanded_ground_velocity_mps: commanded_ground_velocity,
            ground_velocity_was_saturated: commanded_ground_velocity.abs()
                >= self.config.maximum_ground_velocity,
            ground_acceleration_was_saturated: desired_accelerations
                .iter()
                .any(|value| value.abs() >= self.config.maximum_wheel_acceleration),
        })
    }

    /// Rotate the wheel tangent toward the current planar capture vector, then
    /// reuse the reference-matched longitudinal PI law along that tangent.
    /// The wheel pair supplies yaw through differential rolling velocity; it
    /// does not claim direct lateral force or roll authority.
    #[allow(clippy::too_many_arguments)]
    pub fn emit_planar_capture_accelerations(
        &self,
        timestep_seconds: f64,
        station_position_control_world: Vec3,
        left_wheel_position_control_world: Vec3,
        right_wheel_position_control_world: Vec3,
        wheel_center_velocity_control_world: Vec3,
        center_of_mass_position_control_world: Vec3,
        center_of_mass_velocity_control_world: Vec3,
        center_of_mass_height_m: f64,
        heading_x_control_world: Vec3,
        measured_yaw_rate_rad_s: f64,
        joint_velocity: &[f64],
        config: UpkiePlanarCaptureConfig,
        state: &mut UpkiePlanarCaptureState,
        desired_accelerations: &mut [f64; 2],
    ) -> Option<UpkiePlanarCaptureOutput> {
        let finite_vector = |value: Vec3| value.iter().all(|element| element.is_finite());
        if !timestep_seconds.is_finite()
            || timestep_seconds <= 0.0
            || !finite_vector(station_position_control_world)
            || !finite_vector(left_wheel_position_control_world)
            || !finite_vector(right_wheel_position_control_world)
            || !finite_vector(wheel_center_velocity_control_world)
            || !finite_vector(center_of_mass_position_control_world)
            || !finite_vector(center_of_mass_velocity_control_world)
            || !center_of_mass_height_m.is_finite()
            || !finite_vector(heading_x_control_world)
            || !measured_yaw_rate_rad_s.is_finite()
            || !config.steering_release_capture_error_m.is_finite()
            || !config.steering_full_capture_error_m.is_finite()
            || config.steering_release_capture_error_m < 0.0
            || config.steering_release_capture_error_m >= config.steering_full_capture_error_m
            || !config.heading_gain_rad_s_per_rad.is_finite()
            || config.heading_gain_rad_s_per_rad < 0.0
            || !config.maximum_yaw_rate_rad_s.is_finite()
            || config.maximum_yaw_rate_rad_s <= 0.0
            || !config.yaw_rate_slew_rad_s2.is_finite()
            || config.yaw_rate_slew_rad_s2 <= 0.0
            || !config.yaw_rate_tracking_gain_per_s.is_finite()
            || config.yaw_rate_tracking_gain_per_s <= 0.0
            || !config.maximum_yaw_acceleration_rad_s2.is_finite()
            || config.maximum_yaw_acceleration_rad_s2 <= 0.0
            || !state.commanded_yaw_rate_rad_s.is_finite()
            || !state.steering_direction.is_finite()
            || !matches!(state.steering_direction, -1.0 | 0.0 | 1.0)
        {
            return None;
        }
        let heading_norm = heading_x_control_world.xy().norm();
        if heading_norm <= 1.0e-9 {
            return None;
        }
        let tangent_x = Vec3::new(
            heading_x_control_world.x / heading_norm,
            heading_x_control_world.y / heading_norm,
            0.0,
        );
        let tangent_y = Vec3::new(-tangent_x.y, tangent_x.x, 0.0);
        let wheel_center =
            0.5 * (left_wheel_position_control_world + right_wheel_position_control_world);
        let wheel_track = (left_wheel_position_control_world - right_wheel_position_control_world)
            .dot(&tangent_y)
            .abs();
        if !wheel_track.is_finite() || wheel_track <= 1.0e-3 {
            return None;
        }
        let used_height = center_of_mass_height_m.clamp(
            config.sagittal.minimum_com_height_m,
            config.sagittal.maximum_com_height_m,
        );
        if !used_height.is_finite()
            || used_height <= 0.0
            || !config.sagittal.gravity_mps2.is_finite()
            || config.sagittal.gravity_mps2 <= 0.0
        {
            return None;
        }
        let omega = (config.sagittal.gravity_mps2 / used_height).sqrt();
        let relative_com = center_of_mass_position_control_world - wheel_center;
        let capture_offset = relative_com + center_of_mass_velocity_control_world / omega;
        let longitudinal_capture_error = capture_offset.dot(&tangent_x);
        let lateral_capture_error = capture_offset.dot(&tangent_y);
        let planar_capture_error = capture_offset.xy().norm();
        let lateral_support_margin = 0.5 * wheel_track - lateral_capture_error.abs();
        let fade = ((lateral_capture_error.abs() - config.steering_release_capture_error_m)
            / (config.steering_full_capture_error_m - config.steering_release_capture_error_m))
            .clamp(0.0, 1.0);
        let lateral_pressure = fade * fade * (3.0 - 2.0 * fade);
        // A differential drive can choose either direction along its unoriented
        // rolling line. Folding the longitudinal component keeps the requested
        // heading change inside ±90° and lets the sagittal law drive backward.
        if state.steering_direction == 0.0
            && lateral_capture_error.abs() >= config.steering_release_capture_error_m
        {
            state.steering_direction = lateral_capture_error.signum();
        }
        let desired_heading_error = state.steering_direction
            * lateral_capture_error
                .abs()
                .atan2(longitudinal_capture_error.abs().max(1.0e-6));
        let heading_world_rad = tangent_x.y.atan2(tangent_x.x);
        let unsaturated_target_yaw_rate =
            lateral_pressure * config.heading_gain_rad_s_per_rad * desired_heading_error;
        let target_yaw_rate = unsaturated_target_yaw_rate.clamp(
            -config.maximum_yaw_rate_rad_s,
            config.maximum_yaw_rate_rad_s,
        );
        let maximum_rate_step = config.yaw_rate_slew_rad_s2 * timestep_seconds;
        state.commanded_yaw_rate_rad_s += (target_yaw_rate - state.commanded_yaw_rate_rad_s)
            .clamp(-maximum_rate_step, maximum_rate_step);
        let commanded_yaw_acceleration = (config.yaw_rate_tracking_gain_per_s
            * (state.commanded_yaw_rate_rad_s - measured_yaw_rate_rad_s))
            .clamp(
                -config.maximum_yaw_acceleration_rad_s2,
                config.maximum_yaw_acceleration_rad_s2,
            );
        let station_longitudinal = (station_position_control_world - wheel_center).dot(&tangent_x);
        let longitudinal_com = relative_com.dot(&tangent_x);
        let longitudinal_com_velocity = center_of_mass_velocity_control_world.dot(&tangent_x);
        let longitudinal_ground_velocity = wheel_center_velocity_control_world.dot(&tangent_x);
        let sagittal = self.emit_capture_accelerations(
            timestep_seconds,
            station_longitudinal,
            0.0,
            longitudinal_ground_velocity,
            longitudinal_com,
            longitudinal_com_velocity,
            center_of_mass_height_m,
            joint_velocity,
            config.sagittal,
            &mut state.sagittal,
            desired_accelerations,
        )?;
        for (row, desired_acceleration) in desired_accelerations.iter_mut().enumerate() {
            let coordinate = self.coordinates[row];
            let yaw_offset = if coordinate == self.left_coordinate {
                -0.5 * wheel_track * state.commanded_yaw_rate_rad_s
            } else {
                0.5 * wheel_track * state.commanded_yaw_rate_rad_s
            };
            let target_ground_velocity = sagittal.commanded_ground_velocity_mps + yaw_offset;
            let target_wheel_velocity =
                -target_ground_velocity / self.rolling_velocity_coefficients[row];
            *desired_acceleration = (self.config.wheel_velocity_bandwidth
                * (target_wheel_velocity - joint_velocity[coordinate]))
                .clamp(
                    -self.config.maximum_wheel_acceleration,
                    self.config.maximum_wheel_acceleration,
                );
        }
        Some(UpkiePlanarCaptureOutput {
            sagittal,
            heading_world_rad,
            longitudinal_capture_error_m: longitudinal_capture_error,
            lateral_capture_error_m: lateral_capture_error,
            planar_capture_error_m: planar_capture_error,
            lateral_support_margin_m: lateral_support_margin,
            lateral_capture_pressure: lateral_pressure,
            planar_capture_pressure: sagittal.capture_pressure.max(lateral_pressure),
            desired_heading_error_rad: desired_heading_error,
            steering_direction: state.steering_direction,
            target_yaw_rate_rad_s: target_yaw_rate,
            commanded_yaw_rate_rad_s: state.commanded_yaw_rate_rad_s,
            commanded_yaw_acceleration_rad_s2: commanded_yaw_acceleration,
            wheel_track_m: wheel_track,
            steering_was_saturated: unsaturated_target_yaw_rate.abs()
                >= config.maximum_yaw_rate_rad_s,
        })
    }
}

/// Compile the fixed-slot floating balance policy shared by the browser and
/// native endurance evaluator. Runtime target values remain explicit signal
/// inputs; gains, bounds, priorities, and semantic task slots are immutable.
pub fn compile_floating_balance_policy(
    model: &CompiledModel,
    wheeled_balance: bool,
) -> Result<(CompiledSignalProgram, CompiledTaskProgram)> {
    let signals = CompiledSignalProgram::compile(
        vec![
            SignalOp::InputRotation {
                stable_id: 10_001,
                input: 0,
            },
            SignalOp::InputVector {
                stable_id: 10_002,
                input: 0,
            },
            SignalOp::InputVector {
                stable_id: 10_003,
                input: 1,
            },
            SignalOp::CriticallyDampedSpring {
                stable_id: 10_004,
                source: 1,
                bandwidth_hz: 1.0,
            },
        ],
        vec![
            SignalOutputSpec {
                stable_id: 11_001,
                node: 0,
            },
            SignalOutputSpec {
                stable_id: 11_002,
                node: 3,
            },
            SignalOutputSpec {
                stable_id: 11_003,
                node: 2,
            },
        ],
    )?;
    let mut task_specs = vec![
        TaskSpec::FloatingRootOrientation {
            stable_id: 12_001,
            target_signal: 11_001,
            priority: Priority::Viability,
            weight: 1.0,
            bandwidth_hz: 20.0_f64.sqrt() / (2.0 * std::f64::consts::PI),
            damping_ratio: 8.0 / (2.0 * 20.0_f64.sqrt()),
            maximum_acceleration: 8.0,
        },
        TaskSpec::FloatingRootTranslation {
            stable_id: 12_002,
            target_signal: 11_002,
            horizontal_priority: if wheeled_balance {
                Priority::Preference
            } else {
                Priority::Viability
            },
            height_priority: Priority::Intent,
            horizontal_weight: if wheeled_balance { 4.0 } else { 1.0 },
            height_weight: 1.0,
            horizontal_bandwidth_hz: 6.0_f64.sqrt() / (2.0 * std::f64::consts::PI),
            height_bandwidth_hz: 18.0_f64.sqrt() / (2.0 * std::f64::consts::PI),
            horizontal_damping_ratio: 5.0 / (2.0 * 6.0_f64.sqrt()),
            height_damping_ratio: 9.0 / (2.0 * 18.0_f64.sqrt()),
            maximum_horizontal_acceleration: 4.0,
            maximum_height_acceleration: 4.0,
        },
    ];
    if wheeled_balance {
        task_specs.push(TaskSpec::FloatingCenterOfMass {
            stable_id: 12_003,
            target_signal: 11_003,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 30.0_f64.sqrt() / (2.0 * std::f64::consts::PI),
            damping_ratio: 10.0 / (2.0 * 30.0_f64.sqrt()),
            maximum_acceleration: 8.0,
        });
    }
    let tasks = CompiledTaskProgram::compile(model, &signals, task_specs)?;
    Ok((signals, tasks))
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use bonesaw_core::{MotionProgram, RobotState, TimingSpec, Vec3};

    use super::{
        UpkieCaptureReferenceConfig, UpkieCaptureReferenceState, UpkieFallSafeConfig,
        UpkieFallSafeMode, UpkieFallSafeState, UpkieLateralViabilityConfig,
        UpkieLateralViabilityState, UpkiePlanarCaptureConfig, UpkiePlanarCaptureState,
        UpkieSingleSupportReacquisitionConfig, UpkieSingleSupportReacquisitionState,
        UpkieWheelBalancer, UpkieWheelBalancerState, UpkieWheelLoadReserveConfig,
        UpkieWheelLoadReserveState, step_upkie_fall_safe, step_upkie_lateral_viability,
        step_upkie_single_support_reacquisition, step_upkie_wheel_load_reserve,
        write_upkie_fall_safe_contingency,
    };

    fn official_balancer() -> (UpkieWheelBalancer, RobotState) {
        let model_path =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/upkie/upkie.urdf");
        let program =
            MotionProgram::compile_urdf_file(model_path, TimingSpec::default(), 1).unwrap();
        let robot = RobotState::zeros(&program.model);
        let mut balancer = UpkieWheelBalancer::compile(&program.model, &robot).unwrap();
        balancer.config.wheel_radius = 0.06;
        balancer.config.position_damping = 0.7;
        balancer.config.position_stiffness = 1.6;
        balancer.config.pitch_damping = 1.8;
        balancer.config.pitch_stiffness = 20.0;
        balancer.config.maximum_ground_velocity = 2.0;
        balancer.config.maximum_integral_velocity = 10.0;
        (balancer, robot)
    }

    #[test]
    fn official_wheel_balance_update_order_matches_pinned_trace() {
        let (balancer, robot) = official_balancer();
        let mut state = UpkieWheelBalancerState::default();
        let mut desired_accelerations = [0.0; 2];
        let samples: [(f64, f64, f64); 4] = [
            (0.0, 0.0, -0.0),
            (0.01, 0.02, 0.04508),
            (-0.01, -0.02, -4.300_000_000_000_000_3e-2),
            (0.03, -0.01, 2.239_999_999_999_995_5e-3),
        ];
        for (ground_position, pitch, expected) in samples {
            let actual = balancer.emit_accelerations(
                0.005,
                0.0,
                ground_position,
                pitch,
                robot.v.as_slice(),
                &mut state,
                &mut desired_accelerations,
            );
            assert_eq!(
                actual.to_bits(),
                expected.to_bits(),
                "ground velocity differs for position={ground_position}, pitch={pitch}"
            );
        }
    }

    #[test]
    fn integral_state_has_its_own_upstream_limit() {
        let (balancer, robot) = official_balancer();
        let mut state = UpkieWheelBalancerState::default();
        let mut desired_accelerations = [0.0; 2];
        let ground_velocity = balancer.emit_accelerations(
            1.0,
            0.0,
            0.0,
            1.0,
            robot.v.as_slice(),
            &mut state,
            &mut desired_accelerations,
        );
        assert_eq!(state.integral_velocity, -10.0);
        assert_eq!(ground_velocity, 2.0);
    }

    #[test]
    fn capture_reference_moves_toward_velocity_dependent_capture_state() {
        let (balancer, robot) = official_balancer();
        let mut state = UpkieCaptureReferenceState::default();
        let mut desired_accelerations = [0.0; 2];
        let output = balancer
            .emit_capture_accelerations(
                0.005,
                0.0,
                0.0,
                0.0,
                0.0,
                0.6,
                0.3,
                robot.v.as_slice(),
                UpkieCaptureReferenceConfig::default(),
                &mut state,
                &mut desired_accelerations,
            )
            .unwrap();
        assert!(output.capture_position_control_world_m > 0.08);
        assert!(output.capture_error_m > 0.08);
        assert_eq!(output.station_authority, 0.0);
        assert!(output.commanded_ground_velocity_mps > 0.0);
        assert!(desired_accelerations.iter().any(|value| value.abs() > 0.0));
        assert!(desired_accelerations.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn capture_pressure_releases_station_without_a_step() {
        let (balancer, robot) = official_balancer();
        let config = UpkieCaptureReferenceConfig::default();
        let mut previous_authority = 1.0;
        for sample in 0..=100 {
            let capture_error = sample as f64 * 0.001;
            let mut state = UpkieCaptureReferenceState::default();
            let mut desired_accelerations = [0.0; 2];
            let output = balancer
                .emit_capture_accelerations(
                    0.005,
                    0.5,
                    0.0,
                    0.0,
                    capture_error,
                    0.0,
                    0.3,
                    robot.v.as_slice(),
                    config,
                    &mut state,
                    &mut desired_accelerations,
                )
                .unwrap();
            assert!(output.station_authority <= previous_authority);
            previous_authority = output.station_authority;
        }
        assert_eq!(previous_authority, 0.0);
    }

    #[test]
    fn station_target_cannot_change_full_pressure_capture_command() {
        let (balancer, robot) = official_balancer();
        let run = |station_target| {
            let mut state = UpkieCaptureReferenceState::default();
            let mut desired_accelerations = [0.0; 2];
            let output = balancer
                .emit_capture_accelerations(
                    0.005,
                    station_target,
                    0.0,
                    0.0,
                    0.2,
                    0.0,
                    0.3,
                    robot.v.as_slice(),
                    UpkieCaptureReferenceConfig::default(),
                    &mut state,
                    &mut desired_accelerations,
                )
                .unwrap();
            (output, desired_accelerations)
        };
        let left = run(-10.0);
        let right = run(10.0);
        assert_eq!(left.0.station_authority, 0.0);
        assert_eq!(right.0.station_authority, 0.0);
        assert_eq!(
            left.0.commanded_ground_velocity_mps,
            right.0.commanded_ground_velocity_mps
        );
        assert_eq!(left.1, right.1);
    }

    #[test]
    fn planar_capture_steers_symmetrically_toward_lateral_dcm() {
        let (balancer, robot) = official_balancer();
        let config = UpkiePlanarCaptureConfig::default();
        let run = |lateral_velocity: f64| {
            let mut state = UpkiePlanarCaptureState::default();
            let mut desired_accelerations = [0.0; 2];
            let output = balancer
                .emit_planar_capture_accelerations(
                    0.005,
                    Vec3::zeros(),
                    Vec3::new(0.0, 0.1, 0.0),
                    Vec3::new(0.0, -0.1, 0.0),
                    Vec3::zeros(),
                    Vec3::new(0.0, 0.0, 0.3),
                    Vec3::new(0.0, lateral_velocity, 0.0),
                    0.3,
                    Vec3::x(),
                    0.0,
                    robot.v.as_slice(),
                    config,
                    &mut state,
                    &mut desired_accelerations,
                )
                .unwrap();
            (output, desired_accelerations)
        };
        let left = run(0.3);
        let right = run(-0.3);
        assert_eq!(
            left.0.lateral_capture_error_m.to_bits(),
            (-right.0.lateral_capture_error_m).to_bits()
        );
        assert_eq!(
            left.0.commanded_yaw_rate_rad_s.to_bits(),
            (-right.0.commanded_yaw_rate_rad_s).to_bits()
        );
        assert_eq!(left.0.lateral_capture_pressure, 1.0);
        assert_eq!(right.0.lateral_capture_pressure, 1.0);
        assert_eq!(left.0.steering_direction, 1.0);
        assert_eq!(right.0.steering_direction, -1.0);
        // The URDF wheel joint axes are mirrored, so an in-place physical yaw
        // uses equal-signed joint rates and flips sign with steering direction.
        assert_eq!(left.1[0].to_bits(), left.1[1].to_bits());
        assert_eq!(right.1[0].to_bits(), right.1[1].to_bits());
        assert!(left.1[0] * right.1[0] < 0.0);
        assert_eq!(left.0.wheel_track_m, 0.2);
    }

    #[test]
    fn planar_capture_folds_reverse_heading_and_rejects_degenerate_basis() {
        let (balancer, robot) = official_balancer();
        let mut state = UpkiePlanarCaptureState::default();
        let mut desired_accelerations = [0.0; 2];
        let output = balancer
            .emit_planar_capture_accelerations(
                0.005,
                Vec3::zeros(),
                Vec3::new(0.0, 0.1, 0.0),
                Vec3::new(0.0, -0.1, 0.0),
                Vec3::zeros(),
                Vec3::new(-0.1, 0.04, 0.3),
                Vec3::zeros(),
                0.3,
                Vec3::x(),
                0.0,
                robot.v.as_slice(),
                UpkiePlanarCaptureConfig::default(),
                &mut state,
                &mut desired_accelerations,
            )
            .unwrap();
        assert!(output.desired_heading_error_rad > 0.0);
        assert!(output.desired_heading_error_rad <= std::f64::consts::FRAC_PI_2);
        let before = state;
        assert!(
            balancer
                .emit_planar_capture_accelerations(
                    0.005,
                    Vec3::zeros(),
                    Vec3::new(0.0, 0.1, 0.0),
                    Vec3::new(0.0, -0.1, 0.0),
                    Vec3::zeros(),
                    Vec3::new(0.0, 0.0, 0.3),
                    Vec3::zeros(),
                    0.3,
                    Vec3::z(),
                    0.0,
                    robot.v.as_slice(),
                    UpkiePlanarCaptureConfig::default(),
                    &mut state,
                    &mut desired_accelerations,
                )
                .is_none()
        );
        assert_eq!(state, before);
    }

    #[test]
    fn fall_safe_nominal_is_exact_and_risk_transition_is_slew_bounded() {
        let config = UpkieFallSafeConfig::default();
        let mut state = UpkieFallSafeState::default();
        let nominal =
            step_upkie_fall_safe(0.005, 0.0, 0.0, 0.539, true, config, &mut state).unwrap();
        assert_eq!(nominal.mode, UpkieFallSafeMode::Primary);
        assert_eq!(nominal.raw_risk, 0.0);
        assert_eq!(nominal.primary_authority, 1.0);
        assert_eq!(nominal.limiting_reason_flags, 0);

        let first = step_upkie_fall_safe(
            0.005,
            config.tilt_full_rad,
            0.0,
            0.539,
            true,
            config,
            &mut state,
        )
        .unwrap();
        assert_eq!(first.mode, UpkieFallSafeMode::Degraded);
        assert_eq!(first.raw_risk, 1.0);
        assert_eq!(first.primary_authority, 0.96);
        assert!(first.authority_was_slew_limited);
        for _ in 0..24 {
            step_upkie_fall_safe(
                0.005,
                config.tilt_full_rad,
                0.0,
                0.539,
                true,
                config,
                &mut state,
            )
            .unwrap();
        }
        assert_eq!(state.primary_authority, 0.0);
        assert_eq!(state.mode, UpkieFallSafeMode::Contingency);
    }

    #[test]
    fn fall_safe_solver_lease_holds_then_recovers_continuously() {
        let config = UpkieFallSafeConfig {
            minimum_contingency_hold_s: 0.02,
            ..UpkieFallSafeConfig::default()
        };
        let mut state = UpkieFallSafeState::default();
        let mut output =
            step_upkie_fall_safe(0.005, 0.0, 0.0, 0.539, true, config, &mut state).unwrap();
        for _ in 0..config.solver_full_steps {
            output =
                step_upkie_fall_safe(0.005, 0.0, 0.0, 0.539, false, config, &mut state).unwrap();
        }
        assert_eq!(output.solver_pressure, 1.0);
        assert_eq!(output.fresh_command_authority, 0.0);
        assert!(output.hold_remaining_s > 0.0);
        let authority_at_loss = output.primary_authority;

        let first_recovery =
            step_upkie_fall_safe(0.005, 0.0, 0.0, 0.539, true, config, &mut state).unwrap();
        assert_eq!(first_recovery.solver_pressure, 0.0);
        assert!(first_recovery.primary_authority <= authority_at_loss);
        for _ in 0..300 {
            output =
                step_upkie_fall_safe(0.005, 0.0, 0.0, 0.539, true, config, &mut state).unwrap();
        }
        assert_eq!(output.mode, UpkieFallSafeMode::Primary);
        assert_eq!(output.primary_authority, 1.0);
        assert!(output.transition_count >= 2);
    }

    #[test]
    fn fall_safe_fallen_state_is_latched_and_invalid_input_is_atomic() {
        let config = UpkieFallSafeConfig::default();
        let mut state = UpkieFallSafeState::default();
        let fallen = step_upkie_fall_safe(
            0.005,
            config.fallen_tilt_rad,
            0.0,
            0.539,
            true,
            config,
            &mut state,
        )
        .unwrap();
        assert_eq!(fallen.mode, UpkieFallSafeMode::Fallen);
        assert!(fallen.fallen_latched);
        let still_fallen =
            step_upkie_fall_safe(0.005, 0.0, 0.0, 0.539, true, config, &mut state).unwrap();
        assert_eq!(still_fallen.mode, UpkieFallSafeMode::Fallen);
        assert!(still_fallen.fallen_latched);

        let before = state;
        assert!(
            step_upkie_fall_safe(f64::NAN, 0.0, 0.0, 0.539, true, config, &mut state).is_none()
        );
        assert_eq!(state, before);
        let reset = UpkieFallSafeState::default();
        assert_eq!(reset.mode, UpkieFallSafeMode::Primary);
        assert!(!reset.fallen_latched);
    }

    #[test]
    fn fall_safe_contingency_is_bounded_damping_and_invalid_input_is_atomic() {
        let config = UpkieFallSafeConfig::default();
        let root_twist = [20.0, -2.0, 1.0, 20.0, -2.0, 1.0];
        let joint_velocity = [20.0, -2.0, 1.0];
        let mut angular = [f64::NAN; 3];
        let mut linear = [f64::NAN; 3];
        let mut joint = [f64::NAN; 3];
        assert!(write_upkie_fall_safe_contingency(
            &root_twist,
            &joint_velocity,
            config,
            &mut angular,
            &mut linear,
            &mut joint,
        ));
        assert_eq!(angular, [-80.0, 16.0, -8.0]);
        assert_eq!(linear, [-40.0, 8.0, -4.0]);
        assert_eq!(joint, [-120.0, 16.0, -8.0]);

        let before = (angular, linear, joint);
        assert!(!write_upkie_fall_safe_contingency(
            &[f64::NAN; 6],
            &joint_velocity,
            config,
            &mut angular,
            &mut linear,
            &mut joint,
        ));
        assert_eq!((angular, linear, joint), before);
    }

    #[test]
    fn lateral_viability_is_mirrored_bounded_and_points_back_to_support() {
        let config = UpkieLateralViabilityConfig::default();
        let mut positive_state = UpkieLateralViabilityState::default();
        let positive = step_upkie_lateral_viability(
            0.005,
            0.04,
            0.30,
            0.45,
            0.15,
            0.0,
            0.0,
            config,
            &mut positive_state,
        )
        .unwrap();
        let mut negative_state = UpkieLateralViabilityState::default();
        let negative = step_upkie_lateral_viability(
            0.005,
            -0.04,
            -0.30,
            0.45,
            0.15,
            0.0,
            0.0,
            config,
            &mut negative_state,
        )
        .unwrap();
        assert_eq!(positive.lateral_dcm_m, -negative.lateral_dcm_m);
        assert_eq!(positive.commanded_zmp_m, -negative.commanded_zmp_m);
        assert_eq!(
            positive.commanded_lateral_acceleration_m_s2,
            -negative.commanded_lateral_acceleration_m_s2
        );
        assert_eq!(
            positive.commanded_bank_angle_rad,
            -negative.commanded_bank_angle_rad
        );
        assert!(positive.commanded_lateral_acceleration_m_s2 < 0.0);
        assert!(positive.commanded_bank_angle_rad > 0.0);
        assert!(positive.commanded_zmp_m <= positive.support_limit_m);
        assert!(positive.activation_pressure > 0.0);
    }

    #[test]
    fn lateral_viability_slew_and_invalid_input_are_atomic() {
        let config = UpkieLateralViabilityConfig::default();
        let mut state = UpkieLateralViabilityState::default();
        let first =
            step_upkie_lateral_viability(0.005, 0.2, 2.0, 0.45, 0.15, 0.0, 0.0, config, &mut state)
                .unwrap();
        assert!(first.zmp_was_saturated);
        assert!(first.commanded_lateral_acceleration_m_s2.abs() <= 0.4 + 1.0e-12);
        assert!(first.commanded_bank_angle_rad.abs() <= 0.015 + 1.0e-12);
        let before = state;
        assert!(
            step_upkie_lateral_viability(
                f64::NAN,
                0.0,
                0.0,
                0.45,
                0.15,
                0.0,
                0.0,
                config,
                &mut state,
            )
            .is_none()
        );
        assert_eq!(state, before);
    }

    #[test]
    fn wheel_load_reserve_is_dormant_for_symmetric_support() {
        let mut state = UpkieWheelLoadReserveState::default();
        let output = step_upkie_wheel_load_reserve(
            0.004,
            true,
            [0.18, -0.18],
            [26.0, 26.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            UpkieWheelLoadReserveConfig::default(),
            &mut state,
        )
        .unwrap();
        assert!(output.evidence_available);
        assert!(!output.active);
        assert_eq!(output.support_load_fractions, [0.5, 0.5]);
        assert_eq!(output.activation_pressure, 0.0);
        assert_eq!(output.authority, 0.0);
        assert_eq!(output.commanded_lateral_acceleration_m_s2, 0.0);
        assert_eq!(output.commanded_roll_acceleration_rad_s2, 0.0);
    }

    #[test]
    fn wheel_load_reserve_mirrors_and_brakes_over_the_loaded_support() {
        let config = UpkieWheelLoadReserveConfig::default();
        let mut positive_state = UpkieWheelLoadReserveState::default();
        let mut positive = step_upkie_wheel_load_reserve(
            0.004,
            true,
            [0.18, -0.18],
            [18.0, 34.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut positive_state,
        )
        .unwrap();
        let mut negative_state = UpkieWheelLoadReserveState::default();
        let mut negative = step_upkie_wheel_load_reserve(
            0.004,
            true,
            [-0.18, 0.18],
            [18.0, 34.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut negative_state,
        )
        .unwrap();
        for _ in 0..12 {
            positive = step_upkie_wheel_load_reserve(
                0.004,
                true,
                [0.18, -0.18],
                [18.0, 34.0],
                0.0,
                0.0,
                0.45,
                0.0,
                0.0,
                config,
                &mut positive_state,
            )
            .unwrap();
            negative = step_upkie_wheel_load_reserve(
                0.004,
                true,
                [-0.18, 0.18],
                [18.0, 34.0],
                0.0,
                0.0,
                0.45,
                0.0,
                0.0,
                config,
                &mut negative_state,
            )
            .unwrap();
        }
        assert_eq!(positive.weaker_support_index, 0);
        assert!(positive.restoring_zmp_m < 0.0);
        assert!(positive.commanded_lateral_acceleration_m_s2 > 0.0);
        assert!(positive.commanded_bank_angle_rad < 0.0);
        assert_eq!(positive.authority, negative.authority);
        assert_eq!(positive.restoring_zmp_m, -negative.restoring_zmp_m);
        assert_eq!(
            positive.commanded_lateral_acceleration_m_s2,
            -negative.commanded_lateral_acceleration_m_s2
        );
        assert_eq!(
            positive.commanded_roll_acceleration_rad_s2,
            -negative.commanded_roll_acceleration_rad_s2
        );
    }

    #[test]
    fn wheel_load_reserve_can_fail_closed_without_bilateral_evidence() {
        let mut config = UpkieWheelLoadReserveConfig::default();
        config.require_bilateral_evidence_for_authority = true;
        let mut state = UpkieWheelLoadReserveState::default();
        let mut output = step_upkie_wheel_load_reserve(
            0.02,
            true,
            [0.18, -0.18],
            [12.0, 38.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut state,
        )
        .unwrap();
        assert!(output.active);
        assert!(state.authority > 0.0);
        output = step_upkie_wheel_load_reserve(
            0.02,
            false,
            [0.18, -0.18],
            [0.0, 38.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut state,
        )
        .unwrap();
        assert!(!output.evidence_available);
        assert!(!output.active);
        assert_eq!(output.authority, 0.0);
        assert!(state.authority > 0.0);
    }

    #[test]
    fn wheel_load_reserve_predicts_unloading_and_releases_without_a_jump() {
        let config = UpkieWheelLoadReserveConfig::default();
        let mut state = UpkieWheelLoadReserveState::default();
        step_upkie_wheel_load_reserve(
            0.004,
            true,
            [0.18, -0.18],
            [25.0, 27.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut state,
        )
        .unwrap();
        let mut unloading = step_upkie_wheel_load_reserve(
            0.004,
            true,
            [0.18, -0.18],
            [24.0, 28.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut state,
        )
        .unwrap();
        for _ in 0..12 {
            unloading = step_upkie_wheel_load_reserve(
                0.004,
                true,
                [0.18, -0.18],
                [18.0, 34.0],
                0.0,
                0.0,
                0.45,
                0.0,
                0.0,
                config,
                &mut state,
            )
            .unwrap();
        }
        assert!(unloading.predicted_weaker_load_fraction < unloading.weaker_load_fraction);
        assert!(unloading.activation_pressure > 0.0);
        assert!(unloading.authority > 0.0);
        let authority_before_release = unloading.authority;
        let released = step_upkie_wheel_load_reserve(
            0.004,
            false,
            [0.18, -0.18],
            [0.0, 0.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut state,
        )
        .unwrap();
        assert!(!released.evidence_available);
        assert!(released.authority < authority_before_release);
        assert!(released.authority > 0.0);
        assert!(
            (released.commanded_lateral_acceleration_m_s2
                - unloading.commanded_lateral_acceleration_m_s2)
                .abs()
                <= config.lateral_acceleration_slew_m_s3 * 0.004 + 1.0e-12
        );
    }

    #[test]
    fn wheel_load_reserve_invalid_input_is_atomic() {
        let mut state = UpkieWheelLoadReserveState {
            authority: 0.2,
            ..UpkieWheelLoadReserveState::default()
        };
        let before = state;
        assert!(
            step_upkie_wheel_load_reserve(
                0.004,
                true,
                [0.18, -0.18],
                [f64::NAN, 20.0],
                0.0,
                0.0,
                0.45,
                0.0,
                0.0,
                UpkieWheelLoadReserveConfig::default(),
                &mut state,
            )
            .is_none()
        );
        assert_eq!(state, before);
    }

    #[test]
    fn wheel_load_reserve_collapses_a_spent_dynamic_track_without_error() {
        let mut config = UpkieWheelLoadReserveConfig::default();
        config.support_reserve_m = 0.20;
        let mut state = UpkieWheelLoadReserveState::default();
        let output = step_upkie_wheel_load_reserve(
            0.004,
            true,
            [0.05, -0.05],
            [10.0, 30.0],
            0.0,
            0.0,
            0.45,
            0.0,
            0.0,
            config,
            &mut state,
        )
        .unwrap();
        assert_eq!(output.commanded_zmp_m, 0.0);
        assert_eq!(output.requested_lateral_acceleration_m_s2, 0.0);
    }

    #[test]
    fn single_support_reacquisition_lowers_only_from_exact_measured_support() {
        let mut state = UpkieSingleSupportReacquisitionState::default();
        let mut acceleration = [0.0; 6];
        let jacobian = [0.0, 0.25, 0.20, 0.0, 0.0, 0.0];
        let velocity = [0.0; 6];
        let output = step_upkie_single_support_reacquisition(
            0.02,
            true,
            1,
            0.16,
            0.0,
            &jacobian,
            &velocity,
            UpkieSingleSupportReacquisitionConfig::default(),
            &mut state,
            &mut acceleration,
        )
        .unwrap();
        assert!(output.active);
        assert!(output.authority > 0.0);
        assert!(output.commanded_vertical_acceleration_m_s2 < 0.0);
        assert!(acceleration[1] < 0.0);
        assert!(acceleration[2] < 0.0);

        let before = state;
        let rejected = step_upkie_single_support_reacquisition(
            0.02,
            false,
            1,
            0.16,
            0.0,
            &jacobian,
            &velocity,
            UpkieSingleSupportReacquisitionConfig::default(),
            &mut state,
            &mut acceleration,
        )
        .unwrap();
        assert!(!rejected.active);
        assert!(rejected.authority < before.authority);
        assert!(acceleration.iter().all(|value| *value == 0.0));
    }

    #[test]
    fn single_support_reacquisition_rejects_invalid_evidence_atomically() {
        let mut state = UpkieSingleSupportReacquisitionState {
            authority: 0.4,
            ..UpkieSingleSupportReacquisitionState::default()
        };
        let before = state;
        let mut acceleration = [1.0; 6];
        assert!(
            step_upkie_single_support_reacquisition(
                0.02,
                true,
                3,
                f64::NAN,
                0.0,
                &[0.0; 6],
                &[0.0; 6],
                UpkieSingleSupportReacquisitionConfig::default(),
                &mut state,
                &mut acceleration,
            )
            .is_none()
        );
        assert_eq!(state, before);
        assert_eq!(acceleration, [1.0; 6]);
    }
}
