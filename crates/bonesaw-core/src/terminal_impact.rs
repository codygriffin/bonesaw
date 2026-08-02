//! Allocation-free terminal-impact consequence prediction.
//!
//! This module is intentionally narrower than a fall-recovery policy. It
//! predicts the ballistic time and vertical speed at a declared impact plane,
//! then propagates an already-admitted command's root attitude and joint state
//! with a bounded acceleration hold. The result is a typed proxy, not a
//! collision impulse, injury, recovery, or hardware-safety certificate.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TerminalImpactConfig {
    pub gravity_mps2: f64,
    pub acceleration_hold_s: f64,
    pub tilt_soft_limit_rad: f64,
    pub angular_rate_soft_limit_rad_s: f64,
    pub joint_position_soft_headroom_fraction: f64,
    pub joint_velocity_soft_utilization: f64,
    pub actuator_effort_soft_utilization: f64,
    pub vertical_impact_speed_soft_limit_m_s: f64,
    pub tilt_weight: f64,
    pub angular_rate_weight: f64,
    pub joint_position_weight: f64,
    pub joint_velocity_weight: f64,
    pub actuator_effort_weight: f64,
}

impl Default for TerminalImpactConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: 9.81,
            acceleration_hold_s: 0.06,
            tilt_soft_limit_rad: 45.0_f64.to_radians(),
            angular_rate_soft_limit_rad_s: 4.0,
            joint_position_soft_headroom_fraction: 0.10,
            joint_velocity_soft_utilization: 0.70,
            actuator_effort_soft_utilization: 0.65,
            vertical_impact_speed_soft_limit_m_s: 2.0,
            tilt_weight: 1.0,
            angular_rate_weight: 0.5,
            joint_position_weight: 0.5,
            joint_velocity_weight: 0.25,
            actuator_effort_weight: 0.15,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TerminalImpactState<'a> {
    /// Vertical distance from the observed root origin to the declared impact
    /// plane. The plane is evaluator/model evidence, not inferred here.
    pub root_clearance_m: f64,
    pub root_vertical_velocity_m_s: f64,
    /// `[roll, pitch]` small-angle rotation-vector coordinates in control
    /// world.
    pub root_tilt_rad: [f64; 2],
    pub root_angular_rate_rad_s: [f64; 2],
    pub joint_position_rad: &'a [f64],
    pub joint_velocity_rad_s: &'a [f64],
    /// Infinite pairs denote continuous position coordinates and are omitted
    /// from joint-position pressure.
    pub joint_position_lower_rad: &'a [f64],
    pub joint_position_upper_rad: &'a [f64],
    pub joint_velocity_limit_rad_s: &'a [f64],
}

#[derive(Clone, Copy, Debug)]
pub struct TerminalImpactCandidate<'a> {
    /// Whether the ordinary dynamics/resource path admitted this command.
    /// Withholding is represented by an available zero-acceleration candidate.
    pub available: bool,
    pub root_angular_acceleration_rad_s2: [f64; 2],
    pub joint_acceleration_rad_s2: &'a [f64],
    pub maximum_actuator_effort_utilization: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TerminalImpactScore {
    pub available: bool,
    pub time_to_impact_s: f64,
    pub vertical_impact_velocity_m_s: f64,
    /// `0.5 * vz^2`, in J/kg. Horizontal and rotational energy are not
    /// invented when the caller has not supplied mass/inertia evidence.
    pub vertical_specific_impact_energy_j_kg: f64,
    pub terminal_tilt_rad: f64,
    pub terminal_angular_rate_rad_s: f64,
    pub minimum_terminal_joint_headroom_fraction: f64,
    pub maximum_terminal_joint_velocity_utilization: f64,
    pub impact_speed_pressure: f64,
    pub tilt_pressure: f64,
    pub angular_rate_pressure: f64,
    pub joint_position_pressure: f64,
    pub joint_velocity_pressure: f64,
    pub actuator_effort_pressure: f64,
    pub admission_pressure: f64,
    /// Maximum candidate-dependent harm component. Impact speed is excluded
    /// because all support-free candidates share the same ballistic fall.
    pub maximum_terminal_harm_pressure: f64,
    pub aggregate_score: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConservativeTerminalImpactSelection {
    pub selected_index: usize,
    pub baseline_index: usize,
    pub selected_score: f64,
    pub baseline_score: f64,
    pub maximum_component_regression: f64,
    pub maximum_component_improvement: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TerminalImpactError {
    InvalidConfig,
    Dimension,
    InvalidState,
    InvalidCandidate,
    InvalidSelection,
}

fn positive_finite(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn valid_config(config: TerminalImpactConfig) -> bool {
    positive_finite(config.gravity_mps2)
        && positive_finite(config.acceleration_hold_s)
        && positive_finite(config.tilt_soft_limit_rad)
        && positive_finite(config.angular_rate_soft_limit_rad_s)
        && positive_finite(config.joint_position_soft_headroom_fraction)
        && config.joint_position_soft_headroom_fraction <= 0.5
        && config.joint_velocity_soft_utilization.is_finite()
        && (0.0..1.0).contains(&config.joint_velocity_soft_utilization)
        && config.actuator_effort_soft_utilization.is_finite()
        && (0.0..1.0).contains(&config.actuator_effort_soft_utilization)
        && positive_finite(config.vertical_impact_speed_soft_limit_m_s)
        && [
            config.tilt_weight,
            config.angular_rate_weight,
            config.joint_position_weight,
            config.joint_velocity_weight,
            config.actuator_effort_weight,
        ]
        .iter()
        .all(|weight| weight.is_finite() && *weight >= 0.0)
}

fn soft_upper_pressure(value: f64, soft_limit: f64) -> f64 {
    ((value - soft_limit) / (1.0 - soft_limit)).max(0.0)
}

fn propagate(
    position: f64,
    velocity: f64,
    acceleration: f64,
    time_s: f64,
    hold_s: f64,
) -> (f64, f64) {
    let accelerated_time = time_s.min(hold_s);
    let accelerated_position = position
        + velocity * accelerated_time
        + 0.5 * acceleration * accelerated_time * accelerated_time;
    let accelerated_velocity = velocity + acceleration * accelerated_time;
    let coast_time = (time_s - accelerated_time).max(0.0);
    (
        accelerated_position + accelerated_velocity * coast_time,
        accelerated_velocity,
    )
}

/// Predict one support-free candidate at the ballistic root-impact time.
///
/// The function performs fixed work per supplied joint, does not allocate, and
/// validates every input before returning a score. It does not use a policy or
/// advance a physics engine.
pub fn score_terminal_impact(
    state: TerminalImpactState<'_>,
    candidate: TerminalImpactCandidate<'_>,
    config: TerminalImpactConfig,
) -> Result<TerminalImpactScore, TerminalImpactError> {
    if !valid_config(config) {
        return Err(TerminalImpactError::InvalidConfig);
    }
    let joints = state.joint_position_rad.len();
    if state.joint_velocity_rad_s.len() != joints
        || state.joint_position_lower_rad.len() != joints
        || state.joint_position_upper_rad.len() != joints
        || state.joint_velocity_limit_rad_s.len() != joints
        || candidate.joint_acceleration_rad_s2.len() != joints
    {
        return Err(TerminalImpactError::Dimension);
    }
    if !state.root_clearance_m.is_finite()
        || state.root_clearance_m < 0.0
        || !state.root_vertical_velocity_m_s.is_finite()
        || state
            .root_tilt_rad
            .iter()
            .chain(state.root_angular_rate_rad_s.iter())
            .chain(state.joint_position_rad.iter())
            .chain(state.joint_velocity_rad_s.iter())
            .any(|value| !value.is_finite())
    {
        return Err(TerminalImpactError::InvalidState);
    }
    if candidate
        .root_angular_acceleration_rad_s2
        .iter()
        .chain(candidate.joint_acceleration_rad_s2.iter())
        .any(|value| !value.is_finite())
        || !candidate.maximum_actuator_effort_utilization.is_finite()
        || candidate.maximum_actuator_effort_utilization < 0.0
    {
        return Err(TerminalImpactError::InvalidCandidate);
    }
    for joint in 0..joints {
        let lower = state.joint_position_lower_rad[joint];
        let upper = state.joint_position_upper_rad[joint];
        let continuous = lower == f64::NEG_INFINITY && upper == f64::INFINITY;
        if (!continuous && (!lower.is_finite() || !upper.is_finite() || lower >= upper))
            || !positive_finite(state.joint_velocity_limit_rad_s[joint])
        {
            return Err(TerminalImpactError::InvalidState);
        }
    }

    let discriminant = state.root_vertical_velocity_m_s * state.root_vertical_velocity_m_s
        + 2.0 * config.gravity_mps2 * state.root_clearance_m;
    let time_to_impact_s =
        (state.root_vertical_velocity_m_s + discriminant.sqrt()) / config.gravity_mps2;
    let vertical_impact_velocity_m_s =
        state.root_vertical_velocity_m_s - config.gravity_mps2 * time_to_impact_s;
    let vertical_specific_impact_energy_j_kg =
        0.5 * vertical_impact_velocity_m_s * vertical_impact_velocity_m_s;

    let (terminal_roll, terminal_roll_rate) = propagate(
        state.root_tilt_rad[0],
        state.root_angular_rate_rad_s[0],
        candidate.root_angular_acceleration_rad_s2[0],
        time_to_impact_s,
        config.acceleration_hold_s,
    );
    let (terminal_pitch, terminal_pitch_rate) = propagate(
        state.root_tilt_rad[1],
        state.root_angular_rate_rad_s[1],
        candidate.root_angular_acceleration_rad_s2[1],
        time_to_impact_s,
        config.acceleration_hold_s,
    );
    let terminal_tilt_rad = terminal_roll.hypot(terminal_pitch);
    let terminal_angular_rate_rad_s = terminal_roll_rate.hypot(terminal_pitch_rate);

    let mut minimum_terminal_joint_headroom_fraction = 0.5_f64;
    let mut has_bounded_joint = false;
    let mut maximum_terminal_joint_velocity_utilization = 0.0_f64;
    for joint in 0..joints {
        let (position, velocity) = propagate(
            state.joint_position_rad[joint],
            state.joint_velocity_rad_s[joint],
            candidate.joint_acceleration_rad_s2[joint],
            time_to_impact_s,
            config.acceleration_hold_s,
        );
        let lower = state.joint_position_lower_rad[joint];
        let upper = state.joint_position_upper_rad[joint];
        if lower.is_finite() && upper.is_finite() {
            has_bounded_joint = true;
            let span = upper - lower;
            let fraction = ((position - lower).min(upper - position)) / span;
            minimum_terminal_joint_headroom_fraction =
                minimum_terminal_joint_headroom_fraction.min(fraction);
        }
        maximum_terminal_joint_velocity_utilization = maximum_terminal_joint_velocity_utilization
            .max(velocity.abs() / state.joint_velocity_limit_rad_s[joint]);
    }
    if !has_bounded_joint {
        minimum_terminal_joint_headroom_fraction = 0.5;
    }

    let impact_speed_pressure =
        vertical_impact_velocity_m_s.abs() / config.vertical_impact_speed_soft_limit_m_s;
    let tilt_pressure = terminal_tilt_rad / config.tilt_soft_limit_rad;
    let angular_rate_pressure = terminal_angular_rate_rad_s / config.angular_rate_soft_limit_rad_s;
    let joint_position_pressure = ((config.joint_position_soft_headroom_fraction
        - minimum_terminal_joint_headroom_fraction)
        / config.joint_position_soft_headroom_fraction)
        .max(0.0);
    let joint_velocity_pressure = soft_upper_pressure(
        maximum_terminal_joint_velocity_utilization,
        config.joint_velocity_soft_utilization,
    );
    let actuator_effort_pressure = soft_upper_pressure(
        candidate.maximum_actuator_effort_utilization,
        config.actuator_effort_soft_utilization,
    );
    let admission_pressure = if candidate.available { 0.0 } else { 1.0 };
    let maximum_terminal_harm_pressure = tilt_pressure
        .max(angular_rate_pressure)
        .max(joint_position_pressure)
        .max(joint_velocity_pressure)
        .max(actuator_effort_pressure)
        .max(admission_pressure);
    let aggregate_score = impact_speed_pressure
        + config.tilt_weight * tilt_pressure
        + config.angular_rate_weight * angular_rate_pressure
        + config.joint_position_weight * joint_position_pressure
        + config.joint_velocity_weight * joint_velocity_pressure
        + config.actuator_effort_weight * actuator_effort_pressure
        + admission_pressure;

    Ok(TerminalImpactScore {
        available: candidate.available,
        time_to_impact_s,
        vertical_impact_velocity_m_s,
        vertical_specific_impact_energy_j_kg,
        terminal_tilt_rad,
        terminal_angular_rate_rad_s,
        minimum_terminal_joint_headroom_fraction,
        maximum_terminal_joint_velocity_utilization,
        impact_speed_pressure,
        tilt_pressure,
        angular_rate_pressure,
        joint_position_pressure,
        joint_velocity_pressure,
        actuator_effort_pressure,
        admission_pressure,
        maximum_terminal_harm_pressure,
        aggregate_score,
    })
}

fn candidate_components(score: TerminalImpactScore) -> [f64; 5] {
    [
        score.tilt_pressure,
        score.angular_rate_pressure,
        score.joint_position_pressure,
        score.joint_velocity_pressure,
        score.actuator_effort_pressure,
    ]
}

/// Select an available candidate only when every candidate-dependent terminal
/// harm component stays within `maximum_component_regression` of the baseline
/// and at least one improves by `minimum_component_improvement`.
///
/// The baseline remains selected on exact ties. Among admissible alternatives,
/// lower maximum harm wins, then lower aggregate score, then lower index.
pub fn select_conservative_terminal_impact_candidate(
    scores: &[TerminalImpactScore],
    baseline_index: usize,
    maximum_component_regression: f64,
    minimum_component_improvement: f64,
) -> Result<ConservativeTerminalImpactSelection, TerminalImpactError> {
    if scores.is_empty()
        || baseline_index >= scores.len()
        || !scores[baseline_index].available
        || !maximum_component_regression.is_finite()
        || maximum_component_regression < 0.0
        || !minimum_component_improvement.is_finite()
        || minimum_component_improvement < 0.0
        || scores.iter().any(|score| {
            !score.aggregate_score.is_finite()
                || !score.maximum_terminal_harm_pressure.is_finite()
                || candidate_components(*score)
                    .iter()
                    .any(|component| !component.is_finite())
        })
    {
        return Err(TerminalImpactError::InvalidSelection);
    }
    let baseline = scores[baseline_index];
    let baseline_components = candidate_components(baseline);
    let mut selected_index = baseline_index;
    let mut selected_regression = 0.0_f64;
    let mut selected_improvement = 0.0_f64;
    for (index, score) in scores.iter().copied().enumerate() {
        if index == baseline_index || !score.available {
            continue;
        }
        let components = candidate_components(score);
        let mut maximum_regression = 0.0_f64;
        let mut maximum_improvement = 0.0_f64;
        for component in 0..components.len() {
            maximum_regression =
                maximum_regression.max(components[component] - baseline_components[component]);
            maximum_improvement =
                maximum_improvement.max(baseline_components[component] - components[component]);
        }
        if maximum_regression > maximum_component_regression
            || maximum_improvement < minimum_component_improvement
        {
            continue;
        }
        let selected = scores[selected_index];
        if score.maximum_terminal_harm_pressure < selected.maximum_terminal_harm_pressure
            || (score.maximum_terminal_harm_pressure == selected.maximum_terminal_harm_pressure
                && score.aggregate_score < selected.aggregate_score)
        {
            selected_index = index;
            selected_regression = maximum_regression;
            selected_improvement = maximum_improvement;
        }
    }
    Ok(ConservativeTerminalImpactSelection {
        selected_index,
        baseline_index,
        selected_score: scores[selected_index].aggregate_score,
        baseline_score: baseline.aggregate_score,
        maximum_component_regression: selected_regression,
        maximum_component_improvement: selected_improvement,
    })
}

/// Collapse a fixed candidate × support-hypothesis score table into one
/// conservative score per candidate without allocating.
///
/// Input is candidate-major: all hypotheses for candidate zero, then all
/// hypotheses for candidate one, and so on. A candidate is available only
/// when every declared hypothesis is available. Harm-like fields take their
/// componentwise maximum, joint headroom takes its minimum, and ballistic
/// fields must be exactly identical because every hypothesis is scored from
/// the same observed state and impact plane. Raw terminal fields in an
/// envelope are independent witnesses and need not originate from one
/// hypothesis.
pub fn write_terminal_impact_hypothesis_envelopes(
    hypothesis_scores: &[TerminalImpactScore],
    hypotheses_per_candidate: usize,
    envelopes_out: &mut [TerminalImpactScore],
) -> Result<(), TerminalImpactError> {
    if hypotheses_per_candidate == 0
        || envelopes_out.is_empty()
        || hypothesis_scores.len()
            != envelopes_out
                .len()
                .checked_mul(hypotheses_per_candidate)
                .ok_or(TerminalImpactError::Dimension)?
    {
        return Err(TerminalImpactError::Dimension);
    }
    for candidate in 0..envelopes_out.len() {
        let start = candidate * hypotheses_per_candidate;
        let hypotheses = &hypothesis_scores[start..start + hypotheses_per_candidate];
        let reference = hypotheses[0];
        if hypotheses.iter().any(|score| {
            score.time_to_impact_s != reference.time_to_impact_s
                || score.vertical_impact_velocity_m_s != reference.vertical_impact_velocity_m_s
                || score.vertical_specific_impact_energy_j_kg
                    != reference.vertical_specific_impact_energy_j_kg
                || [
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
                ]
                .iter()
                .any(|value| !value.is_finite())
        }) {
            return Err(TerminalImpactError::InvalidSelection);
        }
        let mut envelope = reference;
        envelope.available = true;
        for score in hypotheses {
            envelope.available &= score.available;
            envelope.terminal_tilt_rad = envelope.terminal_tilt_rad.max(score.terminal_tilt_rad);
            envelope.terminal_angular_rate_rad_s = envelope
                .terminal_angular_rate_rad_s
                .max(score.terminal_angular_rate_rad_s);
            envelope.minimum_terminal_joint_headroom_fraction = envelope
                .minimum_terminal_joint_headroom_fraction
                .min(score.minimum_terminal_joint_headroom_fraction);
            envelope.maximum_terminal_joint_velocity_utilization = envelope
                .maximum_terminal_joint_velocity_utilization
                .max(score.maximum_terminal_joint_velocity_utilization);
            envelope.impact_speed_pressure = envelope
                .impact_speed_pressure
                .max(score.impact_speed_pressure);
            envelope.tilt_pressure = envelope.tilt_pressure.max(score.tilt_pressure);
            envelope.angular_rate_pressure = envelope
                .angular_rate_pressure
                .max(score.angular_rate_pressure);
            envelope.joint_position_pressure = envelope
                .joint_position_pressure
                .max(score.joint_position_pressure);
            envelope.joint_velocity_pressure = envelope
                .joint_velocity_pressure
                .max(score.joint_velocity_pressure);
            envelope.actuator_effort_pressure = envelope
                .actuator_effort_pressure
                .max(score.actuator_effort_pressure);
            envelope.admission_pressure = envelope.admission_pressure.max(score.admission_pressure);
            envelope.maximum_terminal_harm_pressure = envelope
                .maximum_terminal_harm_pressure
                .max(score.maximum_terminal_harm_pressure);
            envelope.aggregate_score = envelope.aggregate_score.max(score.aggregate_score);
        }
        envelopes_out[candidate] = envelope;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const Q: [f64; 3] = [0.0, 0.4, 0.0];
    const V: [f64; 3] = [1.0, -1.0, 2.0];
    const LOWER: [f64; 3] = [-1.0, -1.0, f64::NEG_INFINITY];
    const UPPER: [f64; 3] = [1.0, 1.0, f64::INFINITY];
    const V_LIMIT: [f64; 3] = [8.0, 8.0, 20.0];

    fn state() -> TerminalImpactState<'static> {
        TerminalImpactState {
            root_clearance_m: 0.20,
            root_vertical_velocity_m_s: -0.5,
            root_tilt_rad: [0.25, -0.10],
            root_angular_rate_rad_s: [2.0, -1.0],
            joint_position_rad: &Q,
            joint_velocity_rad_s: &V,
            joint_position_lower_rad: &LOWER,
            joint_position_upper_rad: &UPPER,
            joint_velocity_limit_rad_s: &V_LIMIT,
        }
    }

    fn candidate<'a>(angular: [f64; 2], joint: &'a [f64]) -> TerminalImpactCandidate<'a> {
        TerminalImpactCandidate {
            available: true,
            root_angular_acceleration_rad_s2: angular,
            joint_acceleration_rad_s2: joint,
            maximum_actuator_effort_utilization: 0.4,
        }
    }

    #[test]
    fn ballistic_impact_is_candidate_independent_but_posture_is_not() {
        let zero = [0.0; 3];
        let damp = [-8.0, 8.0, -16.0];
        let baseline = score_terminal_impact(
            state(),
            candidate([0.0; 2], &zero),
            TerminalImpactConfig::default(),
        )
        .unwrap();
        let braked = score_terminal_impact(
            state(),
            candidate([-20.0, 10.0], &damp),
            TerminalImpactConfig::default(),
        )
        .unwrap();
        assert_eq!(baseline.time_to_impact_s, braked.time_to_impact_s);
        assert_eq!(
            baseline.vertical_specific_impact_energy_j_kg,
            braked.vertical_specific_impact_energy_j_kg
        );
        assert!(braked.terminal_angular_rate_rad_s < baseline.terminal_angular_rate_rad_s);
        assert!(
            braked.maximum_terminal_joint_velocity_utilization
                < baseline.maximum_terminal_joint_velocity_utilization
        );
    }

    #[test]
    fn vertical_impact_energy_has_physical_units_and_exact_ballistic_identity() {
        let zero = [0.0; 3];
        let score = score_terminal_impact(
            state(),
            candidate([0.0; 2], &zero),
            TerminalImpactConfig::default(),
        )
        .unwrap();
        let expected_speed_squared = 0.5_f64.powi(2) + 2.0 * 9.81 * 0.20;
        assert!(
            (score.vertical_impact_velocity_m_s.powi(2) - expected_speed_squared).abs() < 1.0e-12
        );
        assert!(
            (score.vertical_specific_impact_energy_j_kg - 0.5 * expected_speed_squared).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn conservative_selector_rejects_tradeoffs_and_unavailable_candidates() {
        let baseline = TerminalImpactScore {
            available: true,
            tilt_pressure: 0.8,
            angular_rate_pressure: 0.8,
            maximum_terminal_harm_pressure: 0.8,
            aggregate_score: 2.0,
            ..TerminalImpactScore::default()
        };
        let tradeoff = TerminalImpactScore {
            available: true,
            tilt_pressure: 0.4,
            angular_rate_pressure: 0.9,
            maximum_terminal_harm_pressure: 0.9,
            aggregate_score: 1.5,
            ..TerminalImpactScore::default()
        };
        let unavailable = TerminalImpactScore {
            available: false,
            tilt_pressure: 0.1,
            angular_rate_pressure: 0.1,
            maximum_terminal_harm_pressure: 1.0,
            aggregate_score: 1.0,
            ..TerminalImpactScore::default()
        };
        let selected = select_conservative_terminal_impact_candidate(
            &[baseline, tradeoff, unavailable],
            0,
            0.0,
            0.01,
        )
        .unwrap();
        assert_eq!(selected.selected_index, 0);
    }

    #[test]
    fn conservative_selector_accepts_pareto_improvement_and_ties_low() {
        let baseline = TerminalImpactScore {
            available: true,
            tilt_pressure: 0.8,
            angular_rate_pressure: 0.8,
            maximum_terminal_harm_pressure: 0.8,
            aggregate_score: 2.0,
            ..TerminalImpactScore::default()
        };
        let improvement = TerminalImpactScore {
            available: true,
            tilt_pressure: 0.6,
            angular_rate_pressure: 0.7,
            maximum_terminal_harm_pressure: 0.7,
            aggregate_score: 1.7,
            ..TerminalImpactScore::default()
        };
        let tied = improvement;
        let selected = select_conservative_terminal_impact_candidate(
            &[baseline, improvement, tied],
            0,
            0.0,
            0.01,
        )
        .unwrap();
        assert_eq!(selected.selected_index, 1);
        assert_eq!(selected.maximum_component_regression, 0.0);
        assert!(selected.maximum_component_improvement >= 0.2 - 1.0e-12);
    }

    #[test]
    fn support_hypothesis_envelope_is_componentwise_and_fail_closed() {
        let ballistic = TerminalImpactScore {
            available: true,
            time_to_impact_s: 0.2,
            vertical_impact_velocity_m_s: -2.0,
            vertical_specific_impact_energy_j_kg: 2.0,
            impact_speed_pressure: 1.0,
            minimum_terminal_joint_headroom_fraction: 0.4,
            ..TerminalImpactScore::default()
        };
        let scores = [
            TerminalImpactScore {
                tilt_pressure: 0.2,
                angular_rate_pressure: 0.8,
                maximum_terminal_harm_pressure: 0.8,
                aggregate_score: 1.8,
                ..ballistic
            },
            TerminalImpactScore {
                tilt_pressure: 0.7,
                angular_rate_pressure: 0.3,
                minimum_terminal_joint_headroom_fraction: 0.1,
                maximum_terminal_harm_pressure: 0.7,
                aggregate_score: 1.7,
                ..ballistic
            },
            TerminalImpactScore {
                tilt_pressure: 0.1,
                maximum_terminal_harm_pressure: 0.1,
                aggregate_score: 1.1,
                ..ballistic
            },
            TerminalImpactScore {
                available: false,
                admission_pressure: 1.0,
                maximum_terminal_harm_pressure: 1.0,
                aggregate_score: 2.0,
                ..ballistic
            },
        ];
        let mut envelopes = [TerminalImpactScore::default(); 2];
        write_terminal_impact_hypothesis_envelopes(&scores, 2, &mut envelopes).unwrap();
        assert!(envelopes[0].available);
        assert_eq!(envelopes[0].tilt_pressure, 0.7);
        assert_eq!(envelopes[0].angular_rate_pressure, 0.8);
        assert_eq!(envelopes[0].minimum_terminal_joint_headroom_fraction, 0.1);
        assert_eq!(envelopes[0].aggregate_score, 1.8);
        assert!(!envelopes[1].available);
        assert_eq!(envelopes[1].admission_pressure, 1.0);
    }

    #[test]
    fn support_hypothesis_envelope_rejects_mixed_ballistic_states() {
        let mut scores = [TerminalImpactScore::default(); 2];
        scores[0].available = true;
        scores[0].time_to_impact_s = 0.2;
        scores[1] = scores[0];
        scores[1].time_to_impact_s = 0.3;
        let mut envelope = [TerminalImpactScore::default(); 1];
        assert_eq!(
            write_terminal_impact_hypothesis_envelopes(&scores, 2, &mut envelope),
            Err(TerminalImpactError::InvalidSelection)
        );
    }

    #[test]
    fn invalid_dimensions_and_limits_fail_closed() {
        let zero = [0.0; 2];
        assert_eq!(
            score_terminal_impact(
                state(),
                candidate([0.0; 2], &zero),
                TerminalImpactConfig::default(),
            ),
            Err(TerminalImpactError::Dimension)
        );
        let mut invalid_state = state();
        invalid_state.root_clearance_m = f64::NAN;
        let joint = [0.0; 3];
        assert_eq!(
            score_terminal_impact(
                invalid_state,
                candidate([0.0; 2], &joint),
                TerminalImpactConfig::default(),
            ),
            Err(TerminalImpactError::InvalidState)
        );
    }
}
