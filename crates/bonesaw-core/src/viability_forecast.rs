//! Fixed-work reduced-order viability forecast scoring.
//!
//! This module deliberately does not solve a WBC or advance a physics plant.
//! It scores an already-admitted WBC candidate at a fixed set of future knots,
//! retaining the separation between the exact instantaneous authority query and
//! the slower planner.  The forecast is allocation-free and has no hidden time
//! source or mutable state.

/// Number of future knots in the CPU reference forecast.
pub const VIABILITY_FORECAST_KNOTS: usize = 8;

/// Fixed retained-effort fractions considered when contact evidence is
/// unavailable. Ascending order makes exact score ties fail toward less
/// actuator authority.
pub const INEXACT_OBSERVATION_AUTHORITY_LEVELS_Q15: [u16; 5] = [0, 8_192, 16_384, 24_576, 32_768];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityForecastConfig {
    pub horizon_s: f64,
    pub acceleration_hold_s: f64,
    pub roll_capture_limit_rad: f64,
    pub pitch_capture_limit_rad: f64,
    pub lateral_capture_limit_m: f64,
    pub yaw_capture_limit_rad: f64,
    pub roll_rate_limit_rad_s: f64,
    pub pitch_rate_limit_rad_s: f64,
    pub lateral_velocity_limit_m_s: f64,
    pub yaw_rate_limit_rad_s: f64,
    pub torque_soft_utilization: f64,
    pub joint_soft_headroom_fraction: f64,
    pub terminal_capture_weight: f64,
    pub terminal_rate_weight: f64,
    pub yaw_weight: f64,
    pub resource_weight: f64,
    pub support_weight: f64,
    pub support_recovery_roll_acceleration_rad_s2: f64,
    pub action_weight: f64,
    pub action_delta_weight: f64,
}

impl Default for ViabilityForecastConfig {
    fn default() -> Self {
        Self {
            horizon_s: 0.24,
            acceleration_hold_s: 0.06,
            roll_capture_limit_rad: std::f64::consts::FRAC_PI_4,
            pitch_capture_limit_rad: std::f64::consts::FRAC_PI_4,
            lateral_capture_limit_m: 0.10,
            yaw_capture_limit_rad: std::f64::consts::FRAC_PI_4,
            roll_rate_limit_rad_s: 4.0,
            pitch_rate_limit_rad_s: 4.0,
            lateral_velocity_limit_m_s: 1.0,
            yaw_rate_limit_rad_s: 4.0,
            torque_soft_utilization: 0.65,
            joint_soft_headroom_fraction: 0.20,
            terminal_capture_weight: 0.30,
            terminal_rate_weight: 0.12,
            yaw_weight: 0.08,
            resource_weight: 0.30,
            support_weight: 0.35,
            support_recovery_roll_acceleration_rad_s2: 40.0,
            action_weight: 0.08,
            action_delta_weight: 0.12,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityForecastState {
    pub roll_rad: f64,
    pub roll_rate_rad_s: f64,
    pub pitch_rad: f64,
    pub pitch_rate_rad_s: f64,
    pub lateral_position_m: f64,
    pub lateral_velocity_m_s: f64,
    pub yaw_rad: f64,
    pub yaw_rate_rad_s: f64,
    pub center_of_mass_height_m: f64,
    /// Bit zero is left-wheel support and bit one is right-wheel support.
    pub support_mask: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityForecastCandidate {
    /// Achieved `[roll, lateral, yaw, pitch]` acceleration from the exact WBC.
    pub achieved_acceleration: [f64; 4],
    /// Requested `[roll, lateral, yaw]` acceleration sent to the WBC.
    pub request: [f64; 3],
    /// Request currently executable at the planner boundary.
    pub previous_request: [f64; 3],
    pub maximum_abs_request: [f64; 3],
    pub maximum_torque_utilization: f64,
    pub minimum_joint_headroom_fraction: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityForecastScore {
    pub total: f64,
    pub activation_pressure: f64,
    pub current_capture_pressure: f64,
    pub current_sagittal_pressure: f64,
    pub peak_capture_pressure: f64,
    pub peak_sagittal_pressure: f64,
    pub terminal_capture_pressure: f64,
    pub terminal_sagittal_pressure: f64,
    pub terminal_rate_pressure: f64,
    pub peak_yaw_pressure: f64,
    pub resource_pressure: f64,
    pub action_pressure: f64,
    pub action_delta_pressure: f64,
    pub support_pressure: f64,
    pub minimum_capture_margin: f64,
}

/// State-local, support-free authority selection for one unavailable contact
/// observation. This is a reduced-order forecast witness, not a plant or
/// recovery certificate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InexactObservationAuthoritySelection {
    pub selected_index: u8,
    pub authority_q15: u16,
    pub selected_score: f64,
    pub zero_authority_score: f64,
    pub full_authority_score: f64,
    pub score_improvement_from_zero: f64,
    pub candidate_scores: [f64; INEXACT_OBSERVATION_AUTHORITY_LEVELS_Q15.len()],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ViabilityForecastKnot {
    pub time_s: f64,
    pub roll_rad: f64,
    pub roll_rate_rad_s: f64,
    pub pitch_rad: f64,
    pub pitch_rate_rad_s: f64,
    pub lateral_position_m: f64,
    pub lateral_velocity_m_s: f64,
    pub yaw_rad: f64,
    pub yaw_rate_rad_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViabilityForecastError {
    InvalidConfig,
    InvalidState,
    InvalidCandidate,
}

fn positive_finite(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn valid_config(config: ViabilityForecastConfig) -> bool {
    positive_finite(config.horizon_s)
        && positive_finite(config.acceleration_hold_s)
        && config.acceleration_hold_s <= config.horizon_s
        && positive_finite(config.roll_capture_limit_rad)
        && positive_finite(config.pitch_capture_limit_rad)
        && positive_finite(config.lateral_capture_limit_m)
        && positive_finite(config.yaw_capture_limit_rad)
        && positive_finite(config.roll_rate_limit_rad_s)
        && positive_finite(config.pitch_rate_limit_rad_s)
        && positive_finite(config.lateral_velocity_limit_m_s)
        && positive_finite(config.yaw_rate_limit_rad_s)
        && config.torque_soft_utilization.is_finite()
        && (0.0..1.0).contains(&config.torque_soft_utilization)
        && config.joint_soft_headroom_fraction.is_finite()
        && (0.0..=1.0).contains(&config.joint_soft_headroom_fraction)
        && [
            config.terminal_capture_weight,
            config.terminal_rate_weight,
            config.yaw_weight,
            config.resource_weight,
            config.support_weight,
            config.action_weight,
            config.action_delta_weight,
        ]
        .iter()
        .all(|weight| weight.is_finite() && *weight >= 0.0)
        && positive_finite(config.support_recovery_roll_acceleration_rad_s2)
}

fn apply_bounded_hold(
    position: f64,
    velocity: f64,
    acceleration: f64,
    t: f64,
    hold: f64,
) -> (f64, f64) {
    let accelerated_time = t.min(hold);
    let accelerated_position = position
        + velocity * accelerated_time
        + 0.5 * acceleration * accelerated_time * accelerated_time;
    let accelerated_velocity = velocity + acceleration * accelerated_time;
    let coast_time = (t - hold).max(0.0);
    (
        accelerated_position + accelerated_velocity * coast_time,
        accelerated_velocity,
    )
}

fn normalized_max(values: [f64; 3], limits: [f64; 3]) -> f64 {
    values
        .iter()
        .zip(limits)
        .map(|(value, limit)| value.abs() / limit)
        .fold(0.0, f64::max)
}

/// Write the exact reduced-order path used by [`score_viability_forecast`].
///
/// The output remains untouched when validation fails, which lets callers
/// retain the previous witness without accidentally publishing a partial path.
pub fn predict_viability_forecast_path(
    state: ViabilityForecastState,
    candidate: ViabilityForecastCandidate,
    config: ViabilityForecastConfig,
    output: &mut [ViabilityForecastKnot; VIABILITY_FORECAST_KNOTS],
) -> Result<(), ViabilityForecastError> {
    score_viability_forecast(state, candidate, config)?;
    let mut next = [ViabilityForecastKnot::default(); VIABILITY_FORECAST_KNOTS];
    for (index, knot) in next.iter_mut().enumerate() {
        let time_s = config.horizon_s * (index + 1) as f64 / VIABILITY_FORECAST_KNOTS as f64;
        let (roll, roll_rate) = apply_bounded_hold(
            state.roll_rad,
            state.roll_rate_rad_s,
            candidate.achieved_acceleration[0],
            time_s,
            config.acceleration_hold_s,
        );
        let (pitch, pitch_rate) = apply_bounded_hold(
            state.pitch_rad,
            state.pitch_rate_rad_s,
            candidate.achieved_acceleration[3],
            time_s,
            config.acceleration_hold_s,
        );
        let (lateral_position, lateral_velocity) = apply_bounded_hold(
            state.lateral_position_m,
            state.lateral_velocity_m_s,
            candidate.achieved_acceleration[1],
            time_s,
            config.acceleration_hold_s,
        );
        let (yaw, yaw_rate) = apply_bounded_hold(
            state.yaw_rad,
            state.yaw_rate_rad_s,
            candidate.achieved_acceleration[2],
            time_s,
            config.acceleration_hold_s,
        );
        *knot = ViabilityForecastKnot {
            time_s,
            roll_rad: roll,
            roll_rate_rad_s: roll_rate,
            pitch_rad: pitch,
            pitch_rate_rad_s: pitch_rate,
            lateral_position_m: lateral_position,
            lateral_velocity_m_s: lateral_velocity,
            yaw_rad: yaw,
            yaw_rate_rad_s: yaw_rate,
        };
    }
    *output = next;
    Ok(())
}

/// Score one exact-WBC candidate over a fixed eight-knot reduced-order path.
///
/// Acceleration is held for a bounded prefix and the state then coasts. This is
/// intentionally a conservative local consequence model rather than a claim
/// that the WBC command remains constant for the entire horizon. The returned
/// components stay separate so admission reports cannot hide resource or
/// support pressure inside a scalar verdict.
pub fn score_viability_forecast(
    state: ViabilityForecastState,
    candidate: ViabilityForecastCandidate,
    config: ViabilityForecastConfig,
) -> Result<ViabilityForecastScore, ViabilityForecastError> {
    if !valid_config(config) {
        return Err(ViabilityForecastError::InvalidConfig);
    }
    if ![
        state.roll_rad,
        state.roll_rate_rad_s,
        state.pitch_rad,
        state.pitch_rate_rad_s,
        state.lateral_position_m,
        state.lateral_velocity_m_s,
        state.yaw_rad,
        state.yaw_rate_rad_s,
        state.center_of_mass_height_m,
    ]
    .iter()
    .all(|value| value.is_finite())
        || state.center_of_mass_height_m <= 0.0
        || state.support_mask > 3
    {
        return Err(ViabilityForecastError::InvalidState);
    }
    if candidate
        .achieved_acceleration
        .iter()
        .chain(candidate.request.iter())
        .chain(candidate.previous_request.iter())
        .chain(candidate.maximum_abs_request.iter())
        .any(|value| !value.is_finite())
        || candidate
            .maximum_abs_request
            .iter()
            .any(|value| *value <= 0.0)
        || candidate
            .request
            .iter()
            .enumerate()
            .any(|(axis, value)| value.abs() > candidate.maximum_abs_request[axis] + 1.0e-12)
        || !candidate.maximum_torque_utilization.is_finite()
        || candidate.maximum_torque_utilization < 0.0
        || !candidate.minimum_joint_headroom_fraction.is_finite()
    {
        return Err(ViabilityForecastError::InvalidCandidate);
    }

    let omega = (9.81 / state.center_of_mass_height_m.max(0.05)).sqrt();
    let current_roll_capture = state.roll_rad + state.roll_rate_rad_s / omega;
    let current_pitch_capture = state.pitch_rad + state.pitch_rate_rad_s / omega;
    let current_lateral_capture = state.lateral_position_m + state.lateral_velocity_m_s / omega;
    let current_capture_pressure = (current_roll_capture.abs() / config.roll_capture_limit_rad)
        .max(current_lateral_capture.abs() / config.lateral_capture_limit_m);
    let current_sagittal_pressure = current_pitch_capture.abs() / config.pitch_capture_limit_rad;
    let mut peak_capture_pressure = current_capture_pressure;
    let mut peak_sagittal_pressure = current_sagittal_pressure;
    let mut peak_yaw_pressure = state.yaw_rad.abs() / config.yaw_capture_limit_rad;
    let mut terminal_capture_pressure = current_capture_pressure;
    let mut terminal_sagittal_pressure = current_sagittal_pressure;
    let mut terminal_rate_pressure = (state.roll_rate_rad_s.abs() / config.roll_rate_limit_rad_s)
        .max(state.pitch_rate_rad_s.abs() / config.pitch_rate_limit_rad_s)
        .max(state.lateral_velocity_m_s.abs() / config.lateral_velocity_limit_m_s)
        .max(state.yaw_rate_rad_s.abs() / config.yaw_rate_limit_rad_s);

    for knot in 1..=VIABILITY_FORECAST_KNOTS {
        let t = config.horizon_s * knot as f64 / VIABILITY_FORECAST_KNOTS as f64;
        let (roll, roll_rate) = apply_bounded_hold(
            state.roll_rad,
            state.roll_rate_rad_s,
            candidate.achieved_acceleration[0],
            t,
            config.acceleration_hold_s,
        );
        let (pitch, pitch_rate) = apply_bounded_hold(
            state.pitch_rad,
            state.pitch_rate_rad_s,
            candidate.achieved_acceleration[3],
            t,
            config.acceleration_hold_s,
        );
        let (lateral, lateral_velocity) = apply_bounded_hold(
            state.lateral_position_m,
            state.lateral_velocity_m_s,
            candidate.achieved_acceleration[1],
            t,
            config.acceleration_hold_s,
        );
        let (yaw, yaw_rate) = apply_bounded_hold(
            state.yaw_rad,
            state.yaw_rate_rad_s,
            candidate.achieved_acceleration[2],
            t,
            config.acceleration_hold_s,
        );
        let roll_capture = roll + roll_rate / omega;
        let pitch_capture = pitch + pitch_rate / omega;
        let lateral_capture = lateral + lateral_velocity / omega;
        terminal_capture_pressure = (roll_capture.abs() / config.roll_capture_limit_rad)
            .max(lateral_capture.abs() / config.lateral_capture_limit_m);
        terminal_sagittal_pressure = pitch_capture.abs() / config.pitch_capture_limit_rad;
        terminal_rate_pressure = (roll_rate.abs() / config.roll_rate_limit_rad_s)
            .max(pitch_rate.abs() / config.pitch_rate_limit_rad_s)
            .max(lateral_velocity.abs() / config.lateral_velocity_limit_m_s)
            .max(yaw_rate.abs() / config.yaw_rate_limit_rad_s);
        peak_capture_pressure = peak_capture_pressure.max(terminal_capture_pressure);
        peak_sagittal_pressure = peak_sagittal_pressure.max(terminal_sagittal_pressure);
        peak_yaw_pressure = peak_yaw_pressure.max(yaw.abs() / config.yaw_capture_limit_rad);
    }

    let torque_pressure = ((candidate.maximum_torque_utilization - config.torque_soft_utilization)
        / (1.0 - config.torque_soft_utilization))
        .max(0.0);
    let joint_pressure = if config.joint_soft_headroom_fraction > 0.0 {
        ((config.joint_soft_headroom_fraction - candidate.minimum_joint_headroom_fraction)
            / config.joint_soft_headroom_fraction)
            .max(0.0)
    } else {
        0.0
    };
    let resource_pressure = torque_pressure.max(joint_pressure);
    let action_pressure = normalized_max(candidate.request, candidate.maximum_abs_request);
    let action_delta_pressure = normalized_max(
        [
            candidate.request[0] - candidate.previous_request[0],
            candidate.request[1] - candidate.previous_request[1],
            candidate.request[2] - candidate.previous_request[2],
        ],
        candidate.maximum_abs_request,
    );
    let support_pressure = match state.support_mask {
        3 => 0.0,
        // Left-only support: positive roll lowers the missing right wheel.
        1 => {
            let alignment = candidate.achieved_acceleration[0]
                / config.support_recovery_roll_acceleration_rad_s2;
            1.0 - alignment.clamp(0.0, 1.0) + 0.5 * (-alignment).clamp(0.0, 1.0)
        }
        // Right-only support: negative roll lowers the missing left wheel.
        2 => {
            let alignment = -candidate.achieved_acceleration[0]
                / config.support_recovery_roll_acceleration_rad_s2;
            1.0 - alignment.clamp(0.0, 1.0) + 0.5 * (-alignment).clamp(0.0, 1.0)
        }
        _ => 1.0,
    };
    let actionable_support_pressure = if matches!(state.support_mask, 1 | 2) {
        0.10
    } else {
        0.0
    };
    let activation_pressure = current_capture_pressure
        .max(current_sagittal_pressure)
        .max(actionable_support_pressure);
    let total = peak_capture_pressure.max(peak_sagittal_pressure)
        + config.terminal_capture_weight
            * terminal_capture_pressure.max(terminal_sagittal_pressure)
        + config.terminal_rate_weight * terminal_rate_pressure
        + config.yaw_weight * peak_yaw_pressure
        + config.resource_weight * resource_pressure
        + config.support_weight * support_pressure
        + config.action_weight * action_pressure
        + config.action_delta_weight * action_delta_pressure;

    Ok(ViabilityForecastScore {
        total,
        activation_pressure,
        current_capture_pressure,
        current_sagittal_pressure,
        peak_capture_pressure,
        peak_sagittal_pressure,
        terminal_capture_pressure,
        terminal_sagittal_pressure,
        terminal_rate_pressure,
        peak_yaw_pressure,
        resource_pressure,
        action_pressure,
        action_delta_pressure,
        support_pressure,
        minimum_capture_margin: 1.0 - peak_capture_pressure,
    })
}

/// Choose a retained-effort fraction using only the current reduced state and
/// the last admitted command's achieved acceleration/resource witness.
///
/// Contact support is forced to unavailable for every candidate. The previous
/// achieved acceleration and torque utilization are scaled by the same Q15
/// fraction; request penalties are zero because this selector does not author a
/// new task-space request. All candidates use one fixed eight-knot forecast and
/// exact score ties select the lower authority level. A nonzero minimum
/// improvement may withhold a numerically better candidate when its predicted
/// advantage over zero authority is too small to spend as command authority.
pub fn select_inexact_observation_authority(
    mut state: ViabilityForecastState,
    full_authority_achieved_acceleration: [f64; 4],
    full_authority_maximum_torque_utilization: f64,
    minimum_joint_headroom_fraction: f64,
    minimum_score_improvement_from_zero: f64,
    config: ViabilityForecastConfig,
) -> Result<InexactObservationAuthoritySelection, ViabilityForecastError> {
    if !minimum_score_improvement_from_zero.is_finite() || minimum_score_improvement_from_zero < 0.0
    {
        return Err(ViabilityForecastError::InvalidConfig);
    }
    state.support_mask = 0;
    let mut candidate_scores = [0.0; INEXACT_OBSERVATION_AUTHORITY_LEVELS_Q15.len()];
    let mut selected_index = 0usize;
    for (index, authority_q15) in INEXACT_OBSERVATION_AUTHORITY_LEVELS_Q15
        .iter()
        .copied()
        .enumerate()
    {
        let authority = f64::from(authority_q15) / 32_768.0;
        let score = score_viability_forecast(
            state,
            ViabilityForecastCandidate {
                achieved_acceleration: full_authority_achieved_acceleration
                    .map(|value| authority * value),
                request: [0.0; 3],
                previous_request: [0.0; 3],
                maximum_abs_request: [1.0; 3],
                maximum_torque_utilization: authority * full_authority_maximum_torque_utilization,
                minimum_joint_headroom_fraction,
            },
            config,
        )?;
        candidate_scores[index] = score.total;
        if score.total < candidate_scores[selected_index] {
            selected_index = index;
        }
    }
    if candidate_scores[0] - candidate_scores[selected_index] < minimum_score_improvement_from_zero
    {
        selected_index = 0;
    }
    let selected_score = candidate_scores[selected_index];
    Ok(InexactObservationAuthoritySelection {
        selected_index: selected_index as u8,
        authority_q15: INEXACT_OBSERVATION_AUTHORITY_LEVELS_Q15[selected_index],
        selected_score,
        zero_authority_score: candidate_scores[0],
        full_authority_score: candidate_scores[candidate_scores.len() - 1],
        score_improvement_from_zero: candidate_scores[0] - selected_score,
        candidate_scores,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state() -> ViabilityForecastState {
        ViabilityForecastState {
            roll_rad: 0.12,
            roll_rate_rad_s: 0.40,
            pitch_rad: 0.04,
            pitch_rate_rad_s: 0.10,
            lateral_position_m: 0.025,
            lateral_velocity_m_s: 0.20,
            yaw_rad: 0.05,
            yaw_rate_rad_s: 0.10,
            center_of_mass_height_m: 0.55,
            support_mask: 3,
        }
    }

    fn candidate(acceleration: [f64; 4], request: [f64; 3]) -> ViabilityForecastCandidate {
        ViabilityForecastCandidate {
            achieved_acceleration: acceleration,
            request,
            previous_request: [0.0; 3],
            maximum_abs_request: [250.0, 250.0, 80.0],
            maximum_torque_utilization: 0.40,
            minimum_joint_headroom_fraction: 0.80,
        }
    }

    #[test]
    fn recovery_acceleration_improves_the_whole_path_score() {
        let baseline = score_viability_forecast(
            state(),
            candidate([0.0; 4], [0.0; 3]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        let recovery = score_viability_forecast(
            state(),
            candidate([-8.0, -4.0, 0.0, 0.0], [-8.0, -4.0, 0.0]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert!(recovery.total < baseline.total);
        assert!(recovery.terminal_capture_pressure < baseline.terminal_capture_pressure);
    }

    #[test]
    fn resource_and_action_pressure_prevent_free_edge_commands() {
        let moderate = score_viability_forecast(
            state(),
            candidate([-8.0, -4.0, 0.0, 0.0], [-8.0, -4.0, 0.0]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        let mut saturated = candidate([-8.0, -4.0, 0.0, 0.0], [-250.0, -250.0, 80.0]);
        saturated.maximum_torque_utilization = 0.95;
        saturated.minimum_joint_headroom_fraction = 0.05;
        let saturated =
            score_viability_forecast(state(), saturated, ViabilityForecastConfig::default())
                .unwrap();
        assert!(saturated.total > moderate.total);
        assert!(saturated.resource_pressure > moderate.resource_pressure);
        assert!(saturated.action_pressure > moderate.action_pressure);
    }

    #[test]
    fn support_loss_remains_separate_visible_pressure() {
        let mut unsupported = state();
        unsupported.support_mask = 0;
        let score = score_viability_forecast(
            unsupported,
            candidate([0.0; 4], [0.0; 3]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert_eq!(score.support_pressure, 1.0);
        assert!(score.total >= 0.35);
        assert!(score.current_capture_pressure < 1.0);
    }

    #[test]
    fn single_support_scores_only_the_roll_direction_that_closes_it() {
        let mut left_only = state();
        left_only.support_mask = 1;
        let closing = score_viability_forecast(
            left_only,
            candidate([40.0, 0.0, 0.0, 0.0], [40.0, 0.0, 0.0]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        let opening = score_viability_forecast(
            left_only,
            candidate([-40.0, 0.0, 0.0, 0.0], [-40.0, 0.0, 0.0]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert!(closing.support_pressure < opening.support_pressure);
        assert!(closing.activation_pressure >= 0.10);
    }

    #[test]
    fn lateral_recovery_cannot_hide_a_worse_pitch_path() {
        let safe = score_viability_forecast(
            state(),
            candidate([-8.0, -4.0, 0.0, 0.0], [-40.0, -40.0, 0.0]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        let worse_pitch = score_viability_forecast(
            state(),
            candidate([-8.0, -4.0, 0.0, 20.0], [-40.0, -40.0, 0.0]),
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert!(worse_pitch.peak_sagittal_pressure > safe.peak_sagittal_pressure);
        assert!(worse_pitch.total > safe.total);
    }

    #[test]
    fn invalid_inputs_fail_closed() {
        let mut invalid = candidate([0.0; 4], [0.0; 3]);
        invalid.achieved_acceleration[0] = f64::NAN;
        assert_eq!(
            score_viability_forecast(state(), invalid, ViabilityForecastConfig::default()),
            Err(ViabilityForecastError::InvalidCandidate)
        );
    }

    #[test]
    fn unavailable_authority_selects_recovery_and_forces_support_unknown() {
        let selected = select_inexact_observation_authority(
            state(),
            [-8.0, -4.0, 0.0, 0.0],
            0.40,
            0.80,
            0.0,
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert!(selected.authority_q15 > 0);
        assert!(selected.selected_score < selected.zero_authority_score);
        assert_eq!(
            selected.selected_score,
            selected.candidate_scores[selected.selected_index as usize]
        );
    }

    #[test]
    fn unavailable_authority_rejects_harmful_acceleration_and_ties_low() {
        let selected = select_inexact_observation_authority(
            state(),
            [8.0, 4.0, 0.0, 0.0],
            0.40,
            0.80,
            0.0,
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert_eq!(selected.selected_index, 0);
        assert_eq!(selected.authority_q15, 0);
        assert_eq!(selected.score_improvement_from_zero, 0.0);

        let stationary = ViabilityForecastState {
            roll_rad: 0.0,
            roll_rate_rad_s: 0.0,
            pitch_rad: 0.0,
            pitch_rate_rad_s: 0.0,
            lateral_position_m: 0.0,
            lateral_velocity_m_s: 0.0,
            yaw_rad: 0.0,
            yaw_rate_rad_s: 0.0,
            center_of_mass_height_m: 0.55,
            support_mask: 3,
        };
        let tied = select_inexact_observation_authority(
            stationary,
            [0.0; 4],
            0.0,
            1.0,
            0.0,
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert_eq!(tied.authority_q15, 0);
        assert!(
            tied.candidate_scores
                .iter()
                .all(|score| *score == tied.selected_score)
        );
    }

    #[test]
    fn unavailable_authority_invalid_input_is_atomic_error() {
        assert_eq!(
            select_inexact_observation_authority(
                state(),
                [f64::NAN, 0.0, 0.0, 0.0],
                0.4,
                0.8,
                0.0,
                ViabilityForecastConfig::default(),
            ),
            Err(ViabilityForecastError::InvalidCandidate)
        );
    }

    #[test]
    fn unavailable_authority_requires_the_configured_improvement_margin() {
        let unconstrained = select_inexact_observation_authority(
            state(),
            [-8.0, -4.0, 0.0, 0.0],
            0.40,
            0.80,
            0.0,
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert!(unconstrained.score_improvement_from_zero > 0.0);

        let gated = select_inexact_observation_authority(
            state(),
            [-8.0, -4.0, 0.0, 0.0],
            0.40,
            0.80,
            unconstrained.score_improvement_from_zero + f64::EPSILON,
            ViabilityForecastConfig::default(),
        )
        .unwrap();
        assert_eq!(gated.selected_index, 0);
        assert_eq!(gated.authority_q15, 0);
        assert_eq!(gated.selected_score, gated.zero_authority_score);
        assert_eq!(gated.score_improvement_from_zero, 0.0);

        assert_eq!(
            select_inexact_observation_authority(
                state(),
                [-8.0, -4.0, 0.0, 0.0],
                0.40,
                0.80,
                f64::NAN,
                ViabilityForecastConfig::default(),
            ),
            Err(ViabilityForecastError::InvalidConfig)
        );
    }

    #[test]
    fn path_witness_matches_the_bounded_hold_model() {
        let config = ViabilityForecastConfig::default();
        let candidate = candidate([-8.0, -4.0, 2.0, -3.0], [-8.0, -4.0, 2.0]);
        let mut path = [ViabilityForecastKnot::default(); VIABILITY_FORECAST_KNOTS];
        predict_viability_forecast_path(state(), candidate, config, &mut path).unwrap();
        assert_eq!(path[0].time_s, config.horizon_s / 8.0);
        assert_eq!(path[7].time_s, config.horizon_s);
        let expected = apply_bounded_hold(
            state().roll_rad,
            state().roll_rate_rad_s,
            candidate.achieved_acceleration[0],
            config.horizon_s,
            config.acceleration_hold_s,
        );
        assert_eq!((path[7].roll_rad, path[7].roll_rate_rad_s), expected);
    }

    #[test]
    fn invalid_path_input_does_not_mutate_the_previous_witness() {
        let mut path = [ViabilityForecastKnot {
            time_s: 7.0,
            ..ViabilityForecastKnot::default()
        }; VIABILITY_FORECAST_KNOTS];
        let snapshot = path;
        let mut invalid = candidate([0.0; 4], [0.0; 3]);
        invalid.request[1] = f64::INFINITY;
        assert_eq!(
            predict_viability_forecast_path(
                state(),
                invalid,
                ViabilityForecastConfig::default(),
                &mut path,
            ),
            Err(ViabilityForecastError::InvalidCandidate)
        );
        assert_eq!(path, snapshot);
    }
}
