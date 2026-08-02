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

/// Componentwise interval for the velocity coordinates that drive terminal
/// impact scoring. Position, limits, clearance, and the admitted candidate are
/// fixed at the observation being audited.
///
/// This is a box contract: it does not assign probability to any point inside
/// the interval and does not claim that every Cartesian combination is
/// dynamically reachable.
#[derive(Clone, Copy, Debug)]
pub struct TerminalImpactVelocityBoxState<'a> {
    pub root_clearance_m: f64,
    pub root_vertical_velocity_lower_m_s: f64,
    pub root_vertical_velocity_upper_m_s: f64,
    pub root_tilt_rad: [f64; 2],
    pub root_angular_rate_lower_rad_s: [f64; 2],
    pub root_angular_rate_upper_rad_s: [f64; 2],
    pub joint_position_rad: &'a [f64],
    pub joint_velocity_lower_rad_s: &'a [f64],
    pub joint_velocity_upper_rad_s: &'a [f64],
    pub joint_position_lower_rad: &'a [f64],
    pub joint_position_upper_rad: &'a [f64],
    pub joint_velocity_limit_rad_s: &'a [f64],
}

/// Componentwise interval for the complete terminal-impact state at the
/// beginning of the support-free propagation.  Unlike
/// [`TerminalImpactVelocityBoxState`], this also carries position, attitude,
/// and clearance intervals.  A caller can therefore compose a causal
/// contact-law or estimator tube before asking for a terminal consequence
/// bound; no candidate-minus-baseline residual is inferred here.
#[derive(Clone, Copy, Debug)]
pub struct TerminalImpactStateBox<'a> {
    pub root_clearance_lower_m: f64,
    pub root_clearance_upper_m: f64,
    pub root_vertical_velocity_lower_m_s: f64,
    pub root_vertical_velocity_upper_m_s: f64,
    pub root_tilt_lower_rad: [f64; 2],
    pub root_tilt_upper_rad: [f64; 2],
    pub root_angular_rate_lower_rad_s: [f64; 2],
    pub root_angular_rate_upper_rad_s: [f64; 2],
    pub joint_position_lower_state_rad: &'a [f64],
    pub joint_position_upper_state_rad: &'a [f64],
    pub joint_velocity_lower_rad_s: &'a [f64],
    pub joint_velocity_upper_rad_s: &'a [f64],
    pub joint_position_lower_rad: &'a [f64],
    pub joint_position_upper_rad: &'a [f64],
    pub joint_velocity_limit_rad_s: &'a [f64],
}

/// Candidate-minus-baseline terminal-state tube with explicitly shared
/// baseline uncertainty.
///
/// These coordinates are the terminal coordinates *after* the caller's
/// causal contact transition and support-free propagation. The candidate is
/// represented as `baseline + delta`; the same baseline realization is used
/// on both sides of every consequence difference. Ballistic clearance and
/// vertical speed are omitted deliberately because this boundary requires
/// them to be candidate-invariant.
#[derive(Clone, Copy, Debug)]
pub struct TerminalImpactPairedStateTube<'a> {
    pub available: bool,
    pub baseline_tilt_lower_rad: [f64; 2],
    pub baseline_tilt_upper_rad: [f64; 2],
    pub candidate_tilt_delta_lower_rad: [f64; 2],
    pub candidate_tilt_delta_upper_rad: [f64; 2],
    pub baseline_angular_rate_lower_rad_s: [f64; 2],
    pub baseline_angular_rate_upper_rad_s: [f64; 2],
    pub candidate_angular_rate_delta_lower_rad_s: [f64; 2],
    pub candidate_angular_rate_delta_upper_rad_s: [f64; 2],
    pub baseline_joint_position_lower_rad: &'a [f64],
    pub baseline_joint_position_upper_rad: &'a [f64],
    pub candidate_joint_position_delta_lower_rad: &'a [f64],
    pub candidate_joint_position_delta_upper_rad: &'a [f64],
    pub baseline_joint_velocity_lower_rad_s: &'a [f64],
    pub baseline_joint_velocity_upper_rad_s: &'a [f64],
    pub candidate_joint_velocity_delta_lower_rad_s: &'a [f64],
    pub candidate_joint_velocity_delta_upper_rad_s: &'a [f64],
    pub joint_position_limit_lower_rad: &'a [f64],
    pub joint_position_limit_upper_rad: &'a [f64],
    pub joint_velocity_limit_rad_s: &'a [f64],
    pub baseline_actuator_effort_utilization: f64,
    pub candidate_actuator_effort_utilization: f64,
}

/// One correlated, complete terminal-state hypothesis for a baseline and a
/// candidate after the causal contact transition and before support-free
/// propagation.
///
/// Unlike a componentwise tube, an exemplar never combines coordinates from
/// different residual observations. Clearance and vertical velocity are
/// retained, so candidate-dependent ballistic consequence remains visible.
#[derive(Clone, Copy, Debug)]
pub struct TerminalImpactPairedStateExemplar<'a> {
    pub available: bool,
    pub baseline_state: TerminalImpactState<'a>,
    pub candidate_state: TerminalImpactState<'a>,
    /// A caller-owned all-zero vector used to invoke the ordinary point scorer
    /// without allocating after the contact transition has completed.
    pub support_free_joint_acceleration_rad_s2: &'a [f64],
    pub baseline_actuator_effort_utilization: f64,
    pub candidate_actuator_effort_utilization: f64,
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

/// Number of candidate-dependent terminal consequence components used by the
/// paired delta gate. Impact speed is candidate-invariant for this boundary;
/// admission remains the separate `available` gate.
pub const TERMINAL_IMPACT_PAIRED_COMPONENTS: usize = 6;

/// A caller-supplied outer bound on candidate-minus-baseline terminal
/// consequence. The six components are tilt, angular rate, joint position,
/// joint velocity, actuator effort pressure, and raw joint-headroom loss in
/// that order. Negative is an improvement. This representation preserves
/// paired plant uncertainty without pretending baseline and candidate errors
/// are independent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TerminalImpactComponentDeltaBox {
    pub available: bool,
    pub component_lower: [f64; TERMINAL_IMPACT_PAIRED_COMPONENTS],
    pub component_upper: [f64; TERMINAL_IMPACT_PAIRED_COMPONENTS],
    pub aggregate_lower: f64,
    pub aggregate_upper: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConservativeTerminalImpactDeltaSelection {
    pub selected_index: usize,
    pub baseline_index: usize,
    pub maximum_component_delta_upper: f64,
    pub maximum_guaranteed_component_improvement: f64,
    pub aggregate_delta_lower: f64,
    pub aggregate_delta_upper: f64,
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

fn absolute_interval(lower: f64, upper: f64) -> (f64, f64) {
    let minimum = if lower <= 0.0 && upper >= 0.0 {
        0.0
    } else {
        lower.abs().min(upper.abs())
    };
    (minimum, lower.abs().max(upper.abs()))
}

fn vector_pressure_interval(lower: [f64; 2], upper: [f64; 2], scale: f64) -> (f64, f64) {
    let x = absolute_interval(lower[0], upper[0]);
    let y = absolute_interval(lower[1], upper[1]);
    (x.0.hypot(y.0) / scale, x.1.hypot(y.1) / scale)
}

fn paired_vector_pressure_delta_bound(
    baseline_lower: [f64; 2],
    baseline_upper: [f64; 2],
    delta_lower: [f64; 2],
    delta_upper: [f64; 2],
    scale: f64,
) -> (f64, f64) {
    const CELLS_PER_AXIS: usize = 16;
    let mut lower_bound = f64::INFINITY;
    let mut upper_bound = f64::NEG_INFINITY;
    for x in 0..CELLS_PER_AXIS {
        let x_lower = baseline_lower[0]
            + (baseline_upper[0] - baseline_lower[0]) * (x as f64 / CELLS_PER_AXIS as f64);
        let x_upper = if x + 1 == CELLS_PER_AXIS {
            baseline_upper[0]
        } else {
            baseline_lower[0]
                + (baseline_upper[0] - baseline_lower[0]) * ((x + 1) as f64 / CELLS_PER_AXIS as f64)
        };
        for y in 0..CELLS_PER_AXIS {
            let y_lower = baseline_lower[1]
                + (baseline_upper[1] - baseline_lower[1]) * (y as f64 / CELLS_PER_AXIS as f64);
            let y_upper = if y + 1 == CELLS_PER_AXIS {
                baseline_upper[1]
            } else {
                baseline_lower[1]
                    + (baseline_upper[1] - baseline_lower[1])
                        * ((y + 1) as f64 / CELLS_PER_AXIS as f64)
            };
            let baseline = vector_pressure_interval([x_lower, y_lower], [x_upper, y_upper], scale);
            let candidate = vector_pressure_interval(
                [x_lower + delta_lower[0], y_lower + delta_lower[1]],
                [x_upper + delta_upper[0], y_upper + delta_upper[1]],
                scale,
            );
            lower_bound = lower_bound.min(candidate.0 - baseline.1);
            upper_bound = upper_bound.max(candidate.1 - baseline.0);
        }
    }
    (lower_bound, upper_bound)
}

fn paired_scalar_delta_bound(
    baseline_lower: f64,
    baseline_upper: f64,
    delta_lower: f64,
    delta_upper: f64,
    interval_value: impl Fn(f64, f64) -> (f64, f64),
) -> (f64, f64) {
    const CELLS: usize = 64;
    let mut lower_bound = f64::INFINITY;
    let mut upper_bound = f64::NEG_INFINITY;
    for cell in 0..CELLS {
        let lower =
            baseline_lower + (baseline_upper - baseline_lower) * (cell as f64 / CELLS as f64);
        let upper = if cell + 1 == CELLS {
            baseline_upper
        } else {
            baseline_lower + (baseline_upper - baseline_lower) * ((cell + 1) as f64 / CELLS as f64)
        };
        let baseline = interval_value(lower, upper);
        let candidate = interval_value(lower + delta_lower, upper + delta_upper);
        lower_bound = lower_bound.min(candidate.0 - baseline.1);
        upper_bound = upper_bound.max(candidate.1 - baseline.0);
    }
    (lower_bound, upper_bound)
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

fn interval_product(a_lower: f64, a_upper: f64, b_lower: f64, b_upper: f64) -> (f64, f64) {
    let products = [
        a_lower * b_lower,
        a_lower * b_upper,
        a_upper * b_lower,
        a_upper * b_upper,
    ];
    products.iter().copied().fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(lower, upper), value| (lower.min(value), upper.max(value)),
    )
}

fn acceleration_position_coefficient(time_s: f64, hold_s: f64) -> f64 {
    let accelerated_time = time_s.min(hold_s);
    accelerated_time * time_s - 0.5 * accelerated_time * accelerated_time
}

/// Bound the output of [`propagate`] over independent velocity and time
/// intervals. The interval arithmetic deliberately retains dependency
/// over-approximation between the velocity and acceleration terms.
fn propagate_interval_box(
    position_lower: f64,
    position_upper: f64,
    velocity_lower: f64,
    velocity_upper: f64,
    acceleration_lower: f64,
    acceleration_upper: f64,
    time_lower_s: f64,
    time_upper_s: f64,
    hold_s: f64,
) -> ((f64, f64), (f64, f64)) {
    let (velocity_position_lower, velocity_position_upper) =
        interval_product(velocity_lower, velocity_upper, time_lower_s, time_upper_s);
    let coefficient_lower = acceleration_position_coefficient(time_lower_s, hold_s);
    let coefficient_upper = acceleration_position_coefficient(time_upper_s, hold_s);
    let (acceleration_position_lower, acceleration_position_upper) = interval_product(
        acceleration_lower,
        acceleration_upper,
        coefficient_lower,
        coefficient_upper,
    );
    let accelerated_time_lower = time_lower_s.min(hold_s);
    let accelerated_time_upper = time_upper_s.min(hold_s);
    let (acceleration_velocity_lower, acceleration_velocity_upper) = interval_product(
        acceleration_lower,
        acceleration_upper,
        accelerated_time_lower,
        accelerated_time_upper,
    );
    (
        (
            position_lower + velocity_position_lower + acceleration_position_lower,
            position_upper + velocity_position_upper + acceleration_position_upper,
        ),
        (
            velocity_lower + acceleration_velocity_lower,
            velocity_upper + acceleration_velocity_upper,
        ),
    )
}

fn propagate_interval(
    position: f64,
    velocity_lower: f64,
    velocity_upper: f64,
    acceleration: f64,
    time_lower_s: f64,
    time_upper_s: f64,
    hold_s: f64,
) -> ((f64, f64), (f64, f64)) {
    let (velocity_position_lower, velocity_position_upper) =
        interval_product(velocity_lower, velocity_upper, time_lower_s, time_upper_s);
    let coefficient_lower = acceleration_position_coefficient(time_lower_s, hold_s);
    let coefficient_upper = acceleration_position_coefficient(time_upper_s, hold_s);
    let (acceleration_position_lower, acceleration_position_upper) = if acceleration >= 0.0 {
        (
            acceleration * coefficient_lower,
            acceleration * coefficient_upper,
        )
    } else {
        (
            acceleration * coefficient_upper,
            acceleration * coefficient_lower,
        )
    };
    let accelerated_time_lower = time_lower_s.min(hold_s);
    let accelerated_time_upper = time_upper_s.min(hold_s);
    let (acceleration_velocity_lower, acceleration_velocity_upper) = if acceleration >= 0.0 {
        (
            acceleration * accelerated_time_lower,
            acceleration * accelerated_time_upper,
        )
    } else {
        (
            acceleration * accelerated_time_upper,
            acceleration * accelerated_time_lower,
        )
    };
    (
        (
            position + velocity_position_lower + acceleration_position_lower,
            position + velocity_position_upper + acceleration_position_upper,
        ),
        (
            velocity_lower + acceleration_velocity_lower,
            velocity_upper + acceleration_velocity_upper,
        ),
    )
}

fn maximum_absolute(lower: f64, upper: f64) -> f64 {
    lower.abs().max(upper.abs())
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

/// Compute componentwise upper terminal-pressure bounds for every velocity in
/// a caller-supplied box.
///
/// Ballistic impact time is monotone in initial vertical velocity. Remaining
/// coordinates are propagated with conservative interval arithmetic across
/// that time interval. Consequently every point score in the box is bounded
/// by the returned pressure/maximum/aggregate fields, while
/// `minimum_terminal_joint_headroom_fraction` is a lower bound. Component
/// extrema need not occur at one jointly reachable state, so this result is an
/// authority-safe outer bound rather than a predicted trajectory.
pub fn score_terminal_impact_velocity_box_upper(
    state: TerminalImpactVelocityBoxState<'_>,
    candidate: TerminalImpactCandidate<'_>,
    config: TerminalImpactConfig,
) -> Result<TerminalImpactScore, TerminalImpactError> {
    let joints = state.joint_position_rad.len();
    if state.joint_velocity_lower_rad_s.len() != joints
        || state.joint_velocity_upper_rad_s.len() != joints
        || state.joint_position_lower_rad.len() != joints
        || state.joint_position_upper_rad.len() != joints
        || state.joint_velocity_limit_rad_s.len() != joints
        || candidate.joint_acceleration_rad_s2.len() != joints
    {
        return Err(TerminalImpactError::Dimension);
    }
    if state.root_vertical_velocity_lower_m_s > state.root_vertical_velocity_upper_m_s
        || state
            .root_angular_rate_lower_rad_s
            .iter()
            .zip(state.root_angular_rate_upper_rad_s.iter())
            .any(|(lower, upper)| lower > upper)
        || state
            .joint_velocity_lower_rad_s
            .iter()
            .zip(state.joint_velocity_upper_rad_s.iter())
            .any(|(lower, upper)| lower > upper)
    {
        return Err(TerminalImpactError::InvalidState);
    }

    let lower_state = TerminalImpactState {
        root_clearance_m: state.root_clearance_m,
        root_vertical_velocity_m_s: state.root_vertical_velocity_lower_m_s,
        root_tilt_rad: state.root_tilt_rad,
        root_angular_rate_rad_s: state.root_angular_rate_lower_rad_s,
        joint_position_rad: state.joint_position_rad,
        joint_velocity_rad_s: state.joint_velocity_lower_rad_s,
        joint_position_lower_rad: state.joint_position_lower_rad,
        joint_position_upper_rad: state.joint_position_upper_rad,
        joint_velocity_limit_rad_s: state.joint_velocity_limit_rad_s,
    };
    let upper_state = TerminalImpactState {
        root_vertical_velocity_m_s: state.root_vertical_velocity_upper_m_s,
        root_angular_rate_rad_s: state.root_angular_rate_upper_rad_s,
        joint_velocity_rad_s: state.joint_velocity_upper_rad_s,
        ..lower_state
    };
    // These point calls provide complete config/state/candidate validation
    // before any bound-specific arithmetic is performed.
    let lower_score = score_terminal_impact(lower_state, candidate, config)?;
    let upper_score = score_terminal_impact(upper_state, candidate, config)?;
    let time_lower_s = lower_score.time_to_impact_s;
    let time_upper_s = upper_score.time_to_impact_s;

    let mut terminal_tilt_maximum = [0.0; 2];
    let mut terminal_rate_maximum = [0.0; 2];
    for axis in 0..2 {
        let (tilt, rate) = propagate_interval(
            state.root_tilt_rad[axis],
            state.root_angular_rate_lower_rad_s[axis],
            state.root_angular_rate_upper_rad_s[axis],
            candidate.root_angular_acceleration_rad_s2[axis],
            time_lower_s,
            time_upper_s,
            config.acceleration_hold_s,
        );
        terminal_tilt_maximum[axis] = maximum_absolute(tilt.0, tilt.1);
        terminal_rate_maximum[axis] = maximum_absolute(rate.0, rate.1);
    }
    let terminal_tilt_rad = terminal_tilt_maximum[0].hypot(terminal_tilt_maximum[1]);
    let terminal_angular_rate_rad_s = terminal_rate_maximum[0].hypot(terminal_rate_maximum[1]);

    let mut minimum_terminal_joint_headroom_fraction = 0.5_f64;
    let mut has_bounded_joint = false;
    let mut maximum_terminal_joint_velocity_utilization = 0.0_f64;
    for joint in 0..joints {
        let (position, velocity) = propagate_interval(
            state.joint_position_rad[joint],
            state.joint_velocity_lower_rad_s[joint],
            state.joint_velocity_upper_rad_s[joint],
            candidate.joint_acceleration_rad_s2[joint],
            time_lower_s,
            time_upper_s,
            config.acceleration_hold_s,
        );
        let lower = state.joint_position_lower_rad[joint];
        let upper = state.joint_position_upper_rad[joint];
        if lower.is_finite() && upper.is_finite() {
            has_bounded_joint = true;
            let span = upper - lower;
            let lower_endpoint_headroom = ((position.0 - lower).min(upper - position.0)) / span;
            let upper_endpoint_headroom = ((position.1 - lower).min(upper - position.1)) / span;
            minimum_terminal_joint_headroom_fraction = minimum_terminal_joint_headroom_fraction
                .min(lower_endpoint_headroom.min(upper_endpoint_headroom));
        }
        maximum_terminal_joint_velocity_utilization = maximum_terminal_joint_velocity_utilization
            .max(
                maximum_absolute(velocity.0, velocity.1) / state.joint_velocity_limit_rad_s[joint],
            );
    }
    if !has_bounded_joint {
        minimum_terminal_joint_headroom_fraction = 0.5;
    }

    let (vertical_impact_velocity_m_s, vertical_specific_impact_energy_j_kg) = if lower_score
        .vertical_specific_impact_energy_j_kg
        >= upper_score.vertical_specific_impact_energy_j_kg
    {
        (
            lower_score.vertical_impact_velocity_m_s,
            lower_score.vertical_specific_impact_energy_j_kg,
        )
    } else {
        (
            upper_score.vertical_impact_velocity_m_s,
            upper_score.vertical_specific_impact_energy_j_kg,
        )
    };
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
        time_to_impact_s: time_upper_s,
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

/// Conservatively score every state in a complete componentwise terminal
/// state box.
///
/// This is the composition seam for a causal contact-law or estimator tube:
/// callers provide intervals for the state *before* support-free propagation,
/// while the candidate acceleration remains an independently admitted input.
/// Impact time is bounded from clearance/vertical-velocity monotonicity;
/// position, attitude, and rate are propagated with fixed interval arithmetic.
/// The returned pressure fields are upper bounds and joint headroom is a lower
/// bound. No probability, policy, plant step, or authority is implied.
pub fn score_terminal_impact_state_box_upper(
    state: TerminalImpactStateBox<'_>,
    candidate: TerminalImpactCandidate<'_>,
    config: TerminalImpactConfig,
) -> Result<TerminalImpactScore, TerminalImpactError> {
    let joints = state.joint_position_lower_state_rad.len();
    if state.joint_position_upper_state_rad.len() != joints
        || state.joint_velocity_lower_rad_s.len() != joints
        || state.joint_velocity_upper_rad_s.len() != joints
        || state.joint_position_lower_rad.len() != joints
        || state.joint_position_upper_rad.len() != joints
        || state.joint_velocity_limit_rad_s.len() != joints
        || candidate.joint_acceleration_rad_s2.len() != joints
    {
        return Err(TerminalImpactError::Dimension);
    }
    if !state.root_clearance_lower_m.is_finite()
        || !state.root_clearance_upper_m.is_finite()
        || state.root_clearance_lower_m < 0.0
        || state.root_clearance_lower_m > state.root_clearance_upper_m
        || !state.root_vertical_velocity_lower_m_s.is_finite()
        || !state.root_vertical_velocity_upper_m_s.is_finite()
        || state.root_vertical_velocity_lower_m_s > state.root_vertical_velocity_upper_m_s
        || state
            .root_tilt_lower_rad
            .iter()
            .zip(state.root_tilt_upper_rad.iter())
            .chain(
                state
                    .root_angular_rate_lower_rad_s
                    .iter()
                    .zip(state.root_angular_rate_upper_rad_s.iter()),
            )
            .any(|(lower, upper)| !lower.is_finite() || !upper.is_finite() || lower > upper)
        || state
            .joint_position_lower_state_rad
            .iter()
            .zip(state.joint_position_upper_state_rad.iter())
            .chain(
                state
                    .joint_velocity_lower_rad_s
                    .iter()
                    .zip(state.joint_velocity_upper_rad_s.iter()),
            )
            .any(|(lower, upper)| !lower.is_finite() || !upper.is_finite() || lower > upper)
    {
        return Err(TerminalImpactError::InvalidState);
    }

    // Reuse the point scorer's complete config, candidate, limit, and
    // continuous-coordinate validation before performing interval arithmetic.
    let validation_state = TerminalImpactState {
        root_clearance_m: state.root_clearance_lower_m,
        root_vertical_velocity_m_s: state.root_vertical_velocity_lower_m_s,
        root_tilt_rad: state.root_tilt_lower_rad,
        root_angular_rate_rad_s: state.root_angular_rate_lower_rad_s,
        joint_position_rad: state.joint_position_lower_state_rad,
        joint_velocity_rad_s: state.joint_velocity_lower_rad_s,
        joint_position_lower_rad: state.joint_position_lower_rad,
        joint_position_upper_rad: state.joint_position_upper_rad,
        joint_velocity_limit_rad_s: state.joint_velocity_limit_rad_s,
    };
    score_terminal_impact(validation_state, candidate, config)?;

    let impact_time = |clearance: f64, velocity: f64| {
        let discriminant = velocity * velocity + 2.0 * config.gravity_mps2 * clearance;
        let time = (velocity + discriminant.sqrt()) / config.gravity_mps2;
        (time, -discriminant.sqrt())
    };
    let (time_lower_s, _) = impact_time(
        state.root_clearance_lower_m,
        state.root_vertical_velocity_lower_m_s,
    );
    let (time_upper_s, _) = impact_time(
        state.root_clearance_upper_m,
        state.root_vertical_velocity_upper_m_s,
    );
    if !time_lower_s.is_finite()
        || !time_upper_s.is_finite()
        || time_lower_s < 0.0
        || time_lower_s > time_upper_s
    {
        return Err(TerminalImpactError::InvalidState);
    }

    let maximum_vertical_speed = state
        .root_vertical_velocity_lower_m_s
        .abs()
        .max(state.root_vertical_velocity_upper_m_s.abs());
    let impact_discriminant = maximum_vertical_speed * maximum_vertical_speed
        + 2.0 * config.gravity_mps2 * state.root_clearance_upper_m;
    let vertical_impact_speed_m_s = impact_discriminant.sqrt();
    let vertical_impact_velocity_m_s = -vertical_impact_speed_m_s;
    let vertical_specific_impact_energy_j_kg = 0.5 * vertical_impact_speed_m_s.powi(2);

    let mut terminal_tilt_maximum = [0.0; 2];
    let mut terminal_rate_maximum = [0.0; 2];
    for axis in 0..2 {
        let (tilt, rate) = propagate_interval_box(
            state.root_tilt_lower_rad[axis],
            state.root_tilt_upper_rad[axis],
            state.root_angular_rate_lower_rad_s[axis],
            state.root_angular_rate_upper_rad_s[axis],
            candidate.root_angular_acceleration_rad_s2[axis],
            candidate.root_angular_acceleration_rad_s2[axis],
            time_lower_s,
            time_upper_s,
            config.acceleration_hold_s,
        );
        terminal_tilt_maximum[axis] = maximum_absolute(tilt.0, tilt.1);
        terminal_rate_maximum[axis] = maximum_absolute(rate.0, rate.1);
    }
    let terminal_tilt_rad = terminal_tilt_maximum[0].hypot(terminal_tilt_maximum[1]);
    let terminal_angular_rate_rad_s = terminal_rate_maximum[0].hypot(terminal_rate_maximum[1]);

    let mut minimum_terminal_joint_headroom_fraction = 0.5_f64;
    let mut has_bounded_joint = false;
    let mut maximum_terminal_joint_velocity_utilization = 0.0_f64;
    for joint in 0..joints {
        let (position, velocity) = propagate_interval_box(
            state.joint_position_lower_state_rad[joint],
            state.joint_position_upper_state_rad[joint],
            state.joint_velocity_lower_rad_s[joint],
            state.joint_velocity_upper_rad_s[joint],
            candidate.joint_acceleration_rad_s2[joint],
            candidate.joint_acceleration_rad_s2[joint],
            time_lower_s,
            time_upper_s,
            config.acceleration_hold_s,
        );
        let lower = state.joint_position_lower_rad[joint];
        let upper = state.joint_position_upper_rad[joint];
        if lower.is_finite() && upper.is_finite() {
            has_bounded_joint = true;
            let span = upper - lower;
            let lower_endpoint_headroom = ((position.0 - lower).min(upper - position.0)) / span;
            let upper_endpoint_headroom = ((position.1 - lower).min(upper - position.1)) / span;
            minimum_terminal_joint_headroom_fraction = minimum_terminal_joint_headroom_fraction
                .min(lower_endpoint_headroom.min(upper_endpoint_headroom));
        }
        maximum_terminal_joint_velocity_utilization = maximum_terminal_joint_velocity_utilization
            .max(
                maximum_absolute(velocity.0, velocity.1) / state.joint_velocity_limit_rad_s[joint],
            );
    }
    if !has_bounded_joint {
        minimum_terminal_joint_headroom_fraction = 0.5;
    }

    let impact_speed_pressure =
        vertical_impact_speed_m_s / config.vertical_impact_speed_soft_limit_m_s;
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
        time_to_impact_s: time_upper_s,
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

/// Score one complete paired terminal-state exemplar exactly.
///
/// Both states begin after the caller's causal contact transition. The normal
/// point scorer owns the remaining support-free propagation, including each
/// state's clearance and vertical velocity. Subtraction is safe here because
/// the two scores refer to one explicitly paired hypothesis rather than two
/// independent envelopes. The result contains no policy or authority.
pub fn score_terminal_impact_paired_state_exemplar_delta(
    exemplar: TerminalImpactPairedStateExemplar<'_>,
    config: TerminalImpactConfig,
) -> Result<TerminalImpactComponentDeltaBox, TerminalImpactError> {
    let joints = exemplar.baseline_state.joint_position_rad.len();
    if exemplar.candidate_state.joint_position_rad.len() != joints
        || exemplar.support_free_joint_acceleration_rad_s2.len() != joints
        || exemplar
            .support_free_joint_acceleration_rad_s2
            .iter()
            .any(|value| *value != 0.0)
        || !exemplar.baseline_actuator_effort_utilization.is_finite()
        || exemplar.baseline_actuator_effort_utilization < 0.0
        || !exemplar.candidate_actuator_effort_utilization.is_finite()
        || exemplar.candidate_actuator_effort_utilization < 0.0
    {
        return Err(TerminalImpactError::InvalidCandidate);
    }
    let baseline = score_terminal_impact(
        exemplar.baseline_state,
        TerminalImpactCandidate {
            available: true,
            root_angular_acceleration_rad_s2: [0.0; 2],
            joint_acceleration_rad_s2: exemplar.support_free_joint_acceleration_rad_s2,
            maximum_actuator_effort_utilization: exemplar.baseline_actuator_effort_utilization,
        },
        config,
    )?;
    let candidate = score_terminal_impact(
        exemplar.candidate_state,
        TerminalImpactCandidate {
            available: exemplar.available,
            root_angular_acceleration_rad_s2: [0.0; 2],
            joint_acceleration_rad_s2: exemplar.support_free_joint_acceleration_rad_s2,
            maximum_actuator_effort_utilization: exemplar.candidate_actuator_effort_utilization,
        },
        config,
    )?;
    let component = [
        candidate.tilt_pressure - baseline.tilt_pressure,
        candidate.angular_rate_pressure - baseline.angular_rate_pressure,
        candidate.joint_position_pressure - baseline.joint_position_pressure,
        candidate.joint_velocity_pressure - baseline.joint_velocity_pressure,
        candidate.actuator_effort_pressure - baseline.actuator_effort_pressure,
        baseline.minimum_terminal_joint_headroom_fraction
            - candidate.minimum_terminal_joint_headroom_fraction,
    ];
    let aggregate = candidate.aggregate_score - baseline.aggregate_score;
    Ok(TerminalImpactComponentDeltaBox {
        available: exemplar.available,
        component_lower: component,
        component_upper: component,
        aggregate_lower: aggregate,
        aggregate_upper: aggregate,
    })
}

/// Bound candidate-minus-baseline terminal consequence directly over a
/// shared terminal-state tube.
///
/// This is intentionally not implemented by subtracting independent score
/// envelopes. Root vector pressures are evaluated over a fixed partition of
/// the same baseline box, and scalar joint pressures retain the same baseline
/// cell on both sides of every subtraction. The max-over-joints inequality
/// `max(c) - max(b) <= max(c - b)` then yields conservative position and
/// velocity pressure deltas. The result contains no policy or authority.
pub fn bound_terminal_impact_paired_state_delta(
    tube: TerminalImpactPairedStateTube<'_>,
    config: TerminalImpactConfig,
) -> Result<TerminalImpactComponentDeltaBox, TerminalImpactError> {
    let joints = tube.baseline_joint_position_lower_rad.len();
    if tube.baseline_joint_position_upper_rad.len() != joints
        || tube.candidate_joint_position_delta_lower_rad.len() != joints
        || tube.candidate_joint_position_delta_upper_rad.len() != joints
        || tube.baseline_joint_velocity_lower_rad_s.len() != joints
        || tube.baseline_joint_velocity_upper_rad_s.len() != joints
        || tube.candidate_joint_velocity_delta_lower_rad_s.len() != joints
        || tube.candidate_joint_velocity_delta_upper_rad_s.len() != joints
        || tube.joint_position_limit_lower_rad.len() != joints
        || tube.joint_position_limit_upper_rad.len() != joints
        || tube.joint_velocity_limit_rad_s.len() != joints
    {
        return Err(TerminalImpactError::Dimension);
    }
    if !valid_config(config) {
        return Err(TerminalImpactError::InvalidConfig);
    }
    if !tube.baseline_actuator_effort_utilization.is_finite()
        || tube.baseline_actuator_effort_utilization < 0.0
        || !tube.candidate_actuator_effort_utilization.is_finite()
        || tube.candidate_actuator_effort_utilization < 0.0
    {
        return Err(TerminalImpactError::InvalidCandidate);
    }
    let valid_pair =
        |lower: f64, upper: f64| lower.is_finite() && upper.is_finite() && lower <= upper;
    if tube
        .baseline_tilt_lower_rad
        .iter()
        .zip(tube.baseline_tilt_upper_rad)
        .chain(
            tube.baseline_angular_rate_lower_rad_s
                .iter()
                .zip(tube.baseline_angular_rate_upper_rad_s),
        )
        .chain(
            tube.candidate_tilt_delta_lower_rad
                .iter()
                .zip(tube.candidate_tilt_delta_upper_rad),
        )
        .chain(
            tube.candidate_angular_rate_delta_lower_rad_s
                .iter()
                .zip(tube.candidate_angular_rate_delta_upper_rad_s),
        )
        .any(|(lower, upper)| !valid_pair(*lower, upper))
        || tube
            .baseline_joint_position_lower_rad
            .iter()
            .zip(tube.baseline_joint_position_upper_rad)
            .chain(
                tube.candidate_joint_position_delta_lower_rad
                    .iter()
                    .zip(tube.candidate_joint_position_delta_upper_rad),
            )
            .chain(
                tube.baseline_joint_velocity_lower_rad_s
                    .iter()
                    .zip(tube.baseline_joint_velocity_upper_rad_s),
            )
            .chain(
                tube.candidate_joint_velocity_delta_lower_rad_s
                    .iter()
                    .zip(tube.candidate_joint_velocity_delta_upper_rad_s),
            )
            .any(|(lower, upper)| !valid_pair(*lower, *upper))
        || tube
            .joint_position_limit_lower_rad
            .iter()
            .zip(tube.joint_position_limit_upper_rad)
            .any(|(lower, upper)| {
                !((lower.is_finite() && upper.is_finite() && lower < upper)
                    || (lower.is_infinite()
                        && lower.is_sign_negative()
                        && upper.is_infinite()
                        && upper.is_sign_positive()))
            })
        || tube
            .joint_velocity_limit_rad_s
            .iter()
            .any(|limit| !positive_finite(*limit))
    {
        return Err(TerminalImpactError::InvalidState);
    }

    let exact_zero_delta = tube
        .candidate_tilt_delta_lower_rad
        .iter()
        .chain(tube.candidate_tilt_delta_upper_rad.iter())
        .chain(tube.candidate_angular_rate_delta_lower_rad_s.iter())
        .chain(tube.candidate_angular_rate_delta_upper_rad_s.iter())
        .chain(tube.candidate_joint_position_delta_lower_rad.iter())
        .chain(tube.candidate_joint_position_delta_upper_rad.iter())
        .chain(tube.candidate_joint_velocity_delta_lower_rad_s.iter())
        .chain(tube.candidate_joint_velocity_delta_upper_rad_s.iter())
        .all(|value| *value == 0.0)
        && tube.candidate_actuator_effort_utilization == tube.baseline_actuator_effort_utilization;
    if exact_zero_delta {
        return Ok(TerminalImpactComponentDeltaBox {
            available: tube.available,
            component_lower: [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            component_upper: [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            aggregate_lower: 0.0,
            aggregate_upper: 0.0,
        });
    }

    let tilt = paired_vector_pressure_delta_bound(
        tube.baseline_tilt_lower_rad,
        tube.baseline_tilt_upper_rad,
        tube.candidate_tilt_delta_lower_rad,
        tube.candidate_tilt_delta_upper_rad,
        config.tilt_soft_limit_rad,
    );
    let angular_rate = paired_vector_pressure_delta_bound(
        tube.baseline_angular_rate_lower_rad_s,
        tube.baseline_angular_rate_upper_rad_s,
        tube.candidate_angular_rate_delta_lower_rad_s,
        tube.candidate_angular_rate_delta_upper_rad_s,
        config.angular_rate_soft_limit_rad_s,
    );

    let mut joint_position = (0.0, 0.0);
    let mut raw_headroom_loss = (0.0, 0.0);
    let mut bounded_joint = false;
    let mut joint_velocity = (0.0, 0.0);
    let mut velocity_joint = false;
    for joint in 0..joints {
        let lower_limit = tube.joint_position_limit_lower_rad[joint];
        let upper_limit = tube.joint_position_limit_upper_rad[joint];
        if lower_limit.is_finite() {
            let span = upper_limit - lower_limit;
            let center = 0.5 * (lower_limit + upper_limit);
            let position_pressure = paired_scalar_delta_bound(
                tube.baseline_joint_position_lower_rad[joint],
                tube.baseline_joint_position_upper_rad[joint],
                tube.candidate_joint_position_delta_lower_rad[joint],
                tube.candidate_joint_position_delta_upper_rad[joint],
                |lower, upper| {
                    let absolute = absolute_interval(lower - center, upper - center);
                    let pressure = |distance: f64| {
                        ((config.joint_position_soft_headroom_fraction - 0.5 + distance / span)
                            / config.joint_position_soft_headroom_fraction)
                            .max(0.0)
                    };
                    (pressure(absolute.0), pressure(absolute.1))
                },
            );
            let headroom_loss = paired_scalar_delta_bound(
                tube.baseline_joint_position_lower_rad[joint],
                tube.baseline_joint_position_upper_rad[joint],
                tube.candidate_joint_position_delta_lower_rad[joint],
                tube.candidate_joint_position_delta_upper_rad[joint],
                |lower, upper| {
                    let absolute = absolute_interval(lower - center, upper - center);
                    (absolute.0 / span - 0.5, absolute.1 / span - 0.5)
                },
            );
            if !bounded_joint {
                joint_position = position_pressure;
                raw_headroom_loss = headroom_loss;
                bounded_joint = true;
            } else {
                joint_position.0 = joint_position.0.min(position_pressure.0);
                joint_position.1 = joint_position.1.max(position_pressure.1);
                raw_headroom_loss.0 = raw_headroom_loss.0.min(headroom_loss.0);
                raw_headroom_loss.1 = raw_headroom_loss.1.max(headroom_loss.1);
            }
        }

        let limit = tube.joint_velocity_limit_rad_s[joint];
        let velocity_pressure = paired_scalar_delta_bound(
            tube.baseline_joint_velocity_lower_rad_s[joint],
            tube.baseline_joint_velocity_upper_rad_s[joint],
            tube.candidate_joint_velocity_delta_lower_rad_s[joint],
            tube.candidate_joint_velocity_delta_upper_rad_s[joint],
            |lower, upper| {
                let absolute = absolute_interval(lower, upper);
                (
                    soft_upper_pressure(absolute.0 / limit, config.joint_velocity_soft_utilization),
                    soft_upper_pressure(absolute.1 / limit, config.joint_velocity_soft_utilization),
                )
            },
        );
        if !velocity_joint {
            joint_velocity = velocity_pressure;
            velocity_joint = true;
        } else {
            joint_velocity.0 = joint_velocity.0.min(velocity_pressure.0);
            joint_velocity.1 = joint_velocity.1.max(velocity_pressure.1);
        }
    }

    let effort_delta = soft_upper_pressure(
        tube.candidate_actuator_effort_utilization,
        config.actuator_effort_soft_utilization,
    ) - soft_upper_pressure(
        tube.baseline_actuator_effort_utilization,
        config.actuator_effort_soft_utilization,
    );
    let component_lower = [
        tilt.0,
        angular_rate.0,
        joint_position.0,
        joint_velocity.0,
        effort_delta,
        raw_headroom_loss.0,
    ];
    let component_upper = [
        tilt.1,
        angular_rate.1,
        joint_position.1,
        joint_velocity.1,
        effort_delta,
        raw_headroom_loss.1,
    ];
    let aggregate_lower = config.tilt_weight * tilt.0
        + config.angular_rate_weight * angular_rate.0
        + config.joint_position_weight * joint_position.0
        + config.joint_velocity_weight * joint_velocity.0
        + config.actuator_effort_weight * effort_delta;
    let aggregate_upper = config.tilt_weight * tilt.1
        + config.angular_rate_weight * angular_rate.1
        + config.joint_position_weight * joint_position.1
        + config.joint_velocity_weight * joint_velocity.1
        + config.actuator_effort_weight * effort_delta;
    Ok(TerminalImpactComponentDeltaBox {
        available: tube.available,
        component_lower,
        component_upper,
        aggregate_lower,
        aggregate_upper,
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

/// Select a paired terminal-consequence delta box without allocating.
///
/// The baseline row must be the exact available zero delta. A non-baseline
/// candidate is eligible only when every component upper bound *and the
/// aggregate upper bound* stay within `maximum_component_regression`, and at
/// least one component upper bound is at or below
/// `-minimum_component_improvement`. Thus an optimistic lower bound can never
/// promote an action. Among eligible rows, the smallest worst component upper
/// bound wins, followed by aggregate upper bound and index.
pub fn select_conservative_terminal_impact_delta_candidate(
    candidates: &[TerminalImpactComponentDeltaBox],
    baseline_index: usize,
    maximum_component_regression: f64,
    minimum_component_improvement: f64,
) -> Result<ConservativeTerminalImpactDeltaSelection, TerminalImpactError> {
    if candidates.is_empty()
        || baseline_index >= candidates.len()
        || !maximum_component_regression.is_finite()
        || maximum_component_regression < 0.0
        || !minimum_component_improvement.is_finite()
        || minimum_component_improvement < 0.0
        || candidates.iter().any(|candidate| {
            !candidate.aggregate_lower.is_finite()
                || !candidate.aggregate_upper.is_finite()
                || candidate.aggregate_lower > candidate.aggregate_upper
                || candidate
                    .component_lower
                    .iter()
                    .zip(candidate.component_upper)
                    .any(|(lower, upper)| {
                        !lower.is_finite() || !upper.is_finite() || *lower > upper
                    })
        })
    {
        return Err(TerminalImpactError::InvalidSelection);
    }
    let baseline = candidates[baseline_index];
    if !baseline.available
        || baseline.aggregate_lower != 0.0
        || baseline.aggregate_upper != 0.0
        || baseline.component_lower != [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS]
        || baseline.component_upper != [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS]
    {
        return Err(TerminalImpactError::InvalidSelection);
    }

    let mut selected_index = baseline_index;
    let mut selected_maximum_upper = 0.0;
    let mut selected_guaranteed_improvement = 0.0;
    for (index, candidate) in candidates.iter().copied().enumerate() {
        if index == baseline_index || !candidate.available {
            continue;
        }
        let maximum_upper = candidate
            .component_upper
            .into_iter()
            .fold(f64::NEG_INFINITY, f64::max);
        let guaranteed_improvement = -candidate
            .component_upper
            .into_iter()
            .fold(f64::INFINITY, f64::min);
        if maximum_upper > maximum_component_regression
            || candidate.aggregate_upper > maximum_component_regression
            || guaranteed_improvement < minimum_component_improvement
        {
            continue;
        }
        let selected = candidates[selected_index];
        if selected_index == baseline_index
            || maximum_upper < selected_maximum_upper
            || (maximum_upper == selected_maximum_upper
                && candidate.aggregate_upper < selected.aggregate_upper)
        {
            selected_index = index;
            selected_maximum_upper = maximum_upper;
            selected_guaranteed_improvement = guaranteed_improvement;
        }
    }
    let selected = candidates[selected_index];
    Ok(ConservativeTerminalImpactDeltaSelection {
        selected_index,
        baseline_index,
        maximum_component_delta_upper: selected_maximum_upper,
        maximum_guaranteed_component_improvement: selected_guaranteed_improvement,
        aggregate_delta_lower: selected.aggregate_lower,
        aggregate_delta_upper: selected.aggregate_upper,
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
    fn paired_delta_selector_requires_guaranteed_improvement_without_tradeoff() {
        let baseline = TerminalImpactComponentDeltaBox {
            available: true,
            component_lower: [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            component_upper: [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            aggregate_lower: 0.0,
            aggregate_upper: 0.0,
        };
        let optimistic_only = TerminalImpactComponentDeltaBox {
            available: true,
            component_lower: [-0.5, -0.1, -0.1, -0.1, -0.1, -0.1],
            component_upper: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            aggregate_lower: -0.4,
            aggregate_upper: 0.1,
        };
        let guaranteed = TerminalImpactComponentDeltaBox {
            available: true,
            component_lower: [-0.4, -0.2, -0.1, -0.1, -0.1, -0.1],
            component_upper: [-0.2, -0.1, 0.0, 0.0, 0.0, 0.0],
            aggregate_lower: -0.3,
            aggregate_upper: -0.1,
        };
        let selection = select_conservative_terminal_impact_delta_candidate(
            &[baseline, optimistic_only, guaranteed],
            0,
            0.0,
            0.05,
        )
        .unwrap();
        assert_eq!(selection.selected_index, 2);
        assert_eq!(selection.maximum_component_delta_upper, 0.0);
        assert_eq!(selection.maximum_guaranteed_component_improvement, 0.2);
        assert_eq!(selection.aggregate_delta_upper, -0.1);

        let aggregate_regression = TerminalImpactComponentDeltaBox {
            aggregate_upper: 0.01,
            ..guaranteed
        };
        let aggregate_selection = select_conservative_terminal_impact_delta_candidate(
            &[baseline, aggregate_regression],
            0,
            0.0,
            0.05,
        )
        .unwrap();
        assert_eq!(aggregate_selection.selected_index, 0);
    }

    #[test]
    fn paired_delta_selector_is_fail_closed_and_ties_to_lower_index() {
        let baseline = TerminalImpactComponentDeltaBox {
            available: true,
            component_lower: [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            component_upper: [0.0; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            aggregate_lower: 0.0,
            aggregate_upper: 0.0,
        };
        let candidate = TerminalImpactComponentDeltaBox {
            available: true,
            component_lower: [-0.3; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            component_upper: [-0.1; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            aggregate_lower: -0.4,
            aggregate_upper: -0.2,
        };
        let selection = select_conservative_terminal_impact_delta_candidate(
            &[baseline, candidate, candidate],
            0,
            0.0,
            0.05,
        )
        .unwrap();
        assert_eq!(selection.selected_index, 1);

        let invalid_baseline = TerminalImpactComponentDeltaBox {
            aggregate_upper: 0.01,
            ..baseline
        };
        assert_eq!(
            select_conservative_terminal_impact_delta_candidate(
                &[invalid_baseline, candidate],
                0,
                0.0,
                0.05,
            ),
            Err(TerminalImpactError::InvalidSelection)
        );
        let inverted = TerminalImpactComponentDeltaBox {
            component_lower: [0.2; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            component_upper: [0.1; TERMINAL_IMPACT_PAIRED_COMPONENTS],
            ..candidate
        };
        assert_eq!(
            select_conservative_terminal_impact_delta_candidate(
                &[baseline, inverted],
                0,
                0.0,
                0.05,
            ),
            Err(TerminalImpactError::InvalidSelection)
        );
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
    fn velocity_box_score_bounds_dense_point_samples() {
        let joint_velocity_lower = [-1.4, -0.7, -2.3];
        let joint_velocity_upper = [1.9, 1.2, 2.8];
        let joint_acceleration = [3.0, -5.0, 7.0];
        let candidate = candidate([12.0, -9.0], &joint_acceleration);
        let config = TerminalImpactConfig::default();
        let box_state = TerminalImpactVelocityBoxState {
            root_clearance_m: 0.20,
            root_vertical_velocity_lower_m_s: -1.1,
            root_vertical_velocity_upper_m_s: 0.4,
            root_tilt_rad: [0.25, -0.10],
            root_angular_rate_lower_rad_s: [-2.2, -1.4],
            root_angular_rate_upper_rad_s: [2.6, 1.8],
            joint_position_rad: &Q,
            joint_velocity_lower_rad_s: &joint_velocity_lower,
            joint_velocity_upper_rad_s: &joint_velocity_upper,
            joint_position_lower_rad: &LOWER,
            joint_position_upper_rad: &UPPER,
            joint_velocity_limit_rad_s: &V_LIMIT,
        };
        let bound = score_terminal_impact_velocity_box_upper(box_state, candidate, config).unwrap();
        let pick = |lower: f64, upper: f64, digit: usize| match digit {
            0 => lower,
            1 => 0.5 * (lower + upper),
            _ => upper,
        };
        for encoded in 0..3_usize.pow(6) {
            let mut digits = encoded;
            let mut next = || {
                let digit = digits % 3;
                digits /= 3;
                digit
            };
            let vertical = pick(-1.1, 0.4, next());
            let angular = [pick(-2.2, 2.6, next()), pick(-1.4, 1.8, next())];
            let joint_velocity = [
                pick(joint_velocity_lower[0], joint_velocity_upper[0], next()),
                pick(joint_velocity_lower[1], joint_velocity_upper[1], next()),
                pick(joint_velocity_lower[2], joint_velocity_upper[2], next()),
            ];
            let point = score_terminal_impact(
                TerminalImpactState {
                    root_clearance_m: box_state.root_clearance_m,
                    root_vertical_velocity_m_s: vertical,
                    root_tilt_rad: box_state.root_tilt_rad,
                    root_angular_rate_rad_s: angular,
                    joint_position_rad: &Q,
                    joint_velocity_rad_s: &joint_velocity,
                    joint_position_lower_rad: &LOWER,
                    joint_position_upper_rad: &UPPER,
                    joint_velocity_limit_rad_s: &V_LIMIT,
                },
                candidate,
                config,
            )
            .unwrap();
            assert!(point.time_to_impact_s <= bound.time_to_impact_s + 1.0e-12);
            assert!(
                point.vertical_specific_impact_energy_j_kg
                    <= bound.vertical_specific_impact_energy_j_kg + 1.0e-12
            );
            assert!(point.terminal_tilt_rad <= bound.terminal_tilt_rad + 1.0e-12);
            assert!(
                point.terminal_angular_rate_rad_s <= bound.terminal_angular_rate_rad_s + 1.0e-12
            );
            assert!(
                point.minimum_terminal_joint_headroom_fraction + 1.0e-12
                    >= bound.minimum_terminal_joint_headroom_fraction
            );
            assert!(
                point.maximum_terminal_joint_velocity_utilization
                    <= bound.maximum_terminal_joint_velocity_utilization + 1.0e-12
            );
            assert!(
                point.maximum_terminal_harm_pressure
                    <= bound.maximum_terminal_harm_pressure + 1.0e-12
            );
            assert!(point.aggregate_score <= bound.aggregate_score + 1.0e-12);
        }
    }

    #[test]
    fn complete_state_box_bounds_dense_point_samples() {
        let joint_position_lower_state = [-0.40, -0.30, -0.20];
        let joint_position_upper_state = [0.45, 0.35, 0.40];
        let joint_velocity_lower = [-1.4, -0.7, -2.3];
        let joint_velocity_upper = [1.9, 1.2, 2.8];
        let joint_acceleration = [3.0, -5.0, 7.0];
        let candidate = candidate([12.0, -9.0], &joint_acceleration);
        let config = TerminalImpactConfig::default();
        let box_state = TerminalImpactStateBox {
            root_clearance_lower_m: 0.08,
            root_clearance_upper_m: 0.24,
            root_vertical_velocity_lower_m_s: -1.2,
            root_vertical_velocity_upper_m_s: 0.3,
            root_tilt_lower_rad: [-0.20, -0.25],
            root_tilt_upper_rad: [0.30, 0.15],
            root_angular_rate_lower_rad_s: [-2.0, -1.5],
            root_angular_rate_upper_rad_s: [2.5, 1.8],
            joint_position_lower_state_rad: &joint_position_lower_state,
            joint_position_upper_state_rad: &joint_position_upper_state,
            joint_velocity_lower_rad_s: &joint_velocity_lower,
            joint_velocity_upper_rad_s: &joint_velocity_upper,
            joint_position_lower_rad: &LOWER,
            joint_position_upper_rad: &UPPER,
            joint_velocity_limit_rad_s: &V_LIMIT,
        };
        let bound = score_terminal_impact_state_box_upper(box_state, candidate, config).unwrap();
        let mut seed = 0x7e57_1a11_d15c_a11u64;
        let mut sample = |lower: f64, upper: f64| {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let unit = ((seed >> 11) as f64) * (1.0 / ((1_u64 << 53) as f64));
            lower + unit * (upper - lower)
        };
        for _ in 0..4_096 {
            let joint_position: [f64; 3] = std::array::from_fn(|joint| {
                sample(
                    joint_position_lower_state[joint],
                    joint_position_upper_state[joint],
                )
            });
            let joint_velocity: [f64; 3] = std::array::from_fn(|joint| {
                sample(joint_velocity_lower[joint], joint_velocity_upper[joint])
            });
            let point = score_terminal_impact(
                TerminalImpactState {
                    root_clearance_m: sample(0.08, 0.24),
                    root_vertical_velocity_m_s: sample(-1.2, 0.3),
                    root_tilt_rad: [sample(-0.20, 0.30), sample(-0.25, 0.15)],
                    root_angular_rate_rad_s: [sample(-2.0, 2.5), sample(-1.5, 1.8)],
                    joint_position_rad: &joint_position,
                    joint_velocity_rad_s: &joint_velocity,
                    joint_position_lower_rad: &LOWER,
                    joint_position_upper_rad: &UPPER,
                    joint_velocity_limit_rad_s: &V_LIMIT,
                },
                candidate,
                config,
            )
            .unwrap();
            assert!(point.time_to_impact_s <= bound.time_to_impact_s + 1.0e-12);
            assert!(
                point.vertical_specific_impact_energy_j_kg
                    <= bound.vertical_specific_impact_energy_j_kg + 1.0e-12
            );
            assert!(point.terminal_tilt_rad <= bound.terminal_tilt_rad + 1.0e-12);
            assert!(
                point.terminal_angular_rate_rad_s <= bound.terminal_angular_rate_rad_s + 1.0e-12
            );
            assert!(
                point.minimum_terminal_joint_headroom_fraction + 1.0e-12
                    >= bound.minimum_terminal_joint_headroom_fraction
            );
            assert!(
                point.maximum_terminal_joint_velocity_utilization
                    <= bound.maximum_terminal_joint_velocity_utilization + 1.0e-12
            );
            assert!(point.impact_speed_pressure <= bound.impact_speed_pressure + 1.0e-12);
            assert!(point.tilt_pressure <= bound.tilt_pressure + 1.0e-12);
            assert!(point.angular_rate_pressure <= bound.angular_rate_pressure + 1.0e-12);
            assert!(point.joint_position_pressure <= bound.joint_position_pressure + 1.0e-12);
            assert!(point.joint_velocity_pressure <= bound.joint_velocity_pressure + 1.0e-12);
            assert!(
                point.maximum_terminal_harm_pressure
                    <= bound.maximum_terminal_harm_pressure + 1.0e-12
            );
            assert!(point.aggregate_score <= bound.aggregate_score + 1.0e-12);
        }
    }

    #[test]
    fn complete_state_box_rejects_inverted_coordinates() {
        let joint = [0.0; 3];
        let state = TerminalImpactStateBox {
            root_clearance_lower_m: 0.1,
            root_clearance_upper_m: 0.2,
            root_vertical_velocity_lower_m_s: -0.2,
            root_vertical_velocity_upper_m_s: 0.2,
            root_tilt_lower_rad: [0.0; 2],
            root_tilt_upper_rad: [0.0; 2],
            root_angular_rate_lower_rad_s: [0.0; 2],
            root_angular_rate_upper_rad_s: [0.0; 2],
            joint_position_lower_state_rad: &[0.0, 0.1, 0.0],
            joint_position_upper_state_rad: &[0.0, -0.1, 0.0],
            joint_velocity_lower_rad_s: &joint,
            joint_velocity_upper_rad_s: &joint,
            joint_position_lower_rad: &LOWER,
            joint_position_upper_rad: &UPPER,
            joint_velocity_limit_rad_s: &V_LIMIT,
        };
        assert_eq!(
            score_terminal_impact_state_box_upper(
                state,
                candidate([0.0; 2], &joint),
                TerminalImpactConfig::default(),
            ),
            Err(TerminalImpactError::InvalidState)
        );
    }

    #[test]
    fn paired_state_exemplar_preserves_complete_ballistic_pair() {
        let lower = [-1.0, -1.0];
        let upper = [1.0, 1.0];
        let velocity_limit = [4.0, 4.0];
        let baseline_q = [0.1, -0.2];
        let candidate_q = [0.08, -0.18];
        let baseline_v = [0.5, -0.4];
        let candidate_v = [0.4, -0.3];
        let zero = [0.0, 0.0];
        let baseline_state = TerminalImpactState {
            root_clearance_m: 0.4,
            root_vertical_velocity_m_s: -0.3,
            root_tilt_rad: [0.12, -0.08],
            root_angular_rate_rad_s: [0.3, -0.2],
            joint_position_rad: &baseline_q,
            joint_velocity_rad_s: &baseline_v,
            joint_position_lower_rad: &lower,
            joint_position_upper_rad: &upper,
            joint_velocity_limit_rad_s: &velocity_limit,
        };
        let candidate_state = TerminalImpactState {
            root_clearance_m: 0.36,
            root_vertical_velocity_m_s: -0.45,
            root_tilt_rad: [0.09, -0.06],
            root_angular_rate_rad_s: [0.2, -0.1],
            joint_position_rad: &candidate_q,
            joint_velocity_rad_s: &candidate_v,
            joint_position_lower_rad: &lower,
            joint_position_upper_rad: &upper,
            joint_velocity_limit_rad_s: &velocity_limit,
        };
        let config = TerminalImpactConfig::default();
        let baseline = score_terminal_impact(
            baseline_state,
            TerminalImpactCandidate {
                available: true,
                root_angular_acceleration_rad_s2: [0.0; 2],
                joint_acceleration_rad_s2: &zero,
                maximum_actuator_effort_utilization: 0.4,
            },
            config,
        )
        .unwrap();
        let candidate = score_terminal_impact(
            candidate_state,
            TerminalImpactCandidate {
                available: true,
                root_angular_acceleration_rad_s2: [0.0; 2],
                joint_acceleration_rad_s2: &zero,
                maximum_actuator_effort_utilization: 0.3,
            },
            config,
        )
        .unwrap();
        let delta = score_terminal_impact_paired_state_exemplar_delta(
            TerminalImpactPairedStateExemplar {
                available: true,
                baseline_state,
                candidate_state,
                support_free_joint_acceleration_rad_s2: &zero,
                baseline_actuator_effort_utilization: 0.4,
                candidate_actuator_effort_utilization: 0.3,
            },
            config,
        )
        .unwrap();
        let expected = [
            candidate.tilt_pressure - baseline.tilt_pressure,
            candidate.angular_rate_pressure - baseline.angular_rate_pressure,
            candidate.joint_position_pressure - baseline.joint_position_pressure,
            candidate.joint_velocity_pressure - baseline.joint_velocity_pressure,
            candidate.actuator_effort_pressure - baseline.actuator_effort_pressure,
            baseline.minimum_terminal_joint_headroom_fraction
                - candidate.minimum_terminal_joint_headroom_fraction,
        ];
        assert_eq!(delta.component_lower, expected);
        assert_eq!(delta.component_upper, expected);
        assert_eq!(
            delta.aggregate_lower,
            candidate.aggregate_score - baseline.aggregate_score
        );
        assert_eq!(delta.aggregate_upper, delta.aggregate_lower);
        assert_ne!(
            candidate.impact_speed_pressure, baseline.impact_speed_pressure,
            "candidate-dependent ballistic state must not be discarded"
        );
    }

    #[test]
    fn paired_state_tube_bounds_dense_shared_samples() {
        let baseline_q_lower = [-0.45, 0.10, -0.20];
        let baseline_q_upper = [0.35, 0.55, 0.30];
        let delta_q_lower = [-0.18, -0.25, -0.10];
        let delta_q_upper = [0.12, 0.08, 0.16];
        let baseline_v_lower = [-2.0, -1.4, -3.0];
        let baseline_v_upper = [1.3, 1.8, 2.2];
        let delta_v_lower = [-0.8, -0.5, -1.1];
        let delta_v_upper = [0.4, 0.7, 0.6];
        let tube = TerminalImpactPairedStateTube {
            available: true,
            baseline_tilt_lower_rad: [-0.30, -0.22],
            baseline_tilt_upper_rad: [0.28, 0.35],
            candidate_tilt_delta_lower_rad: [-0.12, -0.16],
            candidate_tilt_delta_upper_rad: [0.08, 0.11],
            baseline_angular_rate_lower_rad_s: [-2.4, -1.7],
            baseline_angular_rate_upper_rad_s: [2.0, 2.6],
            candidate_angular_rate_delta_lower_rad_s: [-0.9, -0.6],
            candidate_angular_rate_delta_upper_rad_s: [0.5, 0.8],
            baseline_joint_position_lower_rad: &baseline_q_lower,
            baseline_joint_position_upper_rad: &baseline_q_upper,
            candidate_joint_position_delta_lower_rad: &delta_q_lower,
            candidate_joint_position_delta_upper_rad: &delta_q_upper,
            baseline_joint_velocity_lower_rad_s: &baseline_v_lower,
            baseline_joint_velocity_upper_rad_s: &baseline_v_upper,
            candidate_joint_velocity_delta_lower_rad_s: &delta_v_lower,
            candidate_joint_velocity_delta_upper_rad_s: &delta_v_upper,
            joint_position_limit_lower_rad: &LOWER,
            joint_position_limit_upper_rad: &UPPER,
            joint_velocity_limit_rad_s: &V_LIMIT,
            baseline_actuator_effort_utilization: 0.52,
            candidate_actuator_effort_utilization: 0.74,
        };
        let config = TerminalImpactConfig::default();
        let bound = bound_terminal_impact_paired_state_delta(tube, config).unwrap();
        let mut seed = 0x5a17_e7ab_1e5u64;
        let mut sample = |lower: f64, upper: f64| {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let unit = ((seed >> 11) as f64) * (1.0 / ((1_u64 << 53) as f64));
            lower + unit * (upper - lower)
        };
        for _ in 0..4_096 {
            let baseline_tilt: [f64; 2] = std::array::from_fn(|axis| {
                sample(
                    tube.baseline_tilt_lower_rad[axis],
                    tube.baseline_tilt_upper_rad[axis],
                )
            });
            let tilt_delta: [f64; 2] = std::array::from_fn(|axis| {
                sample(
                    tube.candidate_tilt_delta_lower_rad[axis],
                    tube.candidate_tilt_delta_upper_rad[axis],
                )
            });
            let baseline_rate: [f64; 2] = std::array::from_fn(|axis| {
                sample(
                    tube.baseline_angular_rate_lower_rad_s[axis],
                    tube.baseline_angular_rate_upper_rad_s[axis],
                )
            });
            let rate_delta: [f64; 2] = std::array::from_fn(|axis| {
                sample(
                    tube.candidate_angular_rate_delta_lower_rad_s[axis],
                    tube.candidate_angular_rate_delta_upper_rad_s[axis],
                )
            });
            let baseline_q: [f64; 3] = std::array::from_fn(|joint| {
                sample(baseline_q_lower[joint], baseline_q_upper[joint])
            });
            let candidate_q: [f64; 3] = std::array::from_fn(|joint| {
                baseline_q[joint] + sample(delta_q_lower[joint], delta_q_upper[joint])
            });
            let baseline_v: [f64; 3] = std::array::from_fn(|joint| {
                sample(baseline_v_lower[joint], baseline_v_upper[joint])
            });
            let candidate_v: [f64; 3] = std::array::from_fn(|joint| {
                baseline_v[joint] + sample(delta_v_lower[joint], delta_v_upper[joint])
            });
            let vector_pressure = |value: [f64; 2], limit: f64| value[0].hypot(value[1]) / limit;
            let tilt = vector_pressure(
                [
                    baseline_tilt[0] + tilt_delta[0],
                    baseline_tilt[1] + tilt_delta[1],
                ],
                config.tilt_soft_limit_rad,
            ) - vector_pressure(baseline_tilt, config.tilt_soft_limit_rad);
            let angular_rate =
                vector_pressure(
                    [
                        baseline_rate[0] + rate_delta[0],
                        baseline_rate[1] + rate_delta[1],
                    ],
                    config.angular_rate_soft_limit_rad_s,
                ) - vector_pressure(baseline_rate, config.angular_rate_soft_limit_rad_s);
            let joint_position_pressure = |positions: &[f64; 3]| {
                (0..2)
                    .map(|joint| {
                        let span = UPPER[joint] - LOWER[joint];
                        let headroom = ((positions[joint] - LOWER[joint])
                            .min(UPPER[joint] - positions[joint]))
                            / span;
                        ((config.joint_position_soft_headroom_fraction - headroom)
                            / config.joint_position_soft_headroom_fraction)
                            .max(0.0)
                    })
                    .fold(0.0_f64, f64::max)
            };
            let minimum_headroom = |positions: &[f64; 3]| {
                (0..2)
                    .map(|joint| {
                        let span = UPPER[joint] - LOWER[joint];
                        ((positions[joint] - LOWER[joint]).min(UPPER[joint] - positions[joint]))
                            / span
                    })
                    .fold(0.5_f64, f64::min)
            };
            let velocity_pressure = |velocities: &[f64; 3]| {
                soft_upper_pressure(
                    (0..3)
                        .map(|joint| velocities[joint].abs() / V_LIMIT[joint])
                        .fold(0.0_f64, f64::max),
                    config.joint_velocity_soft_utilization,
                )
            };
            let joint_position =
                joint_position_pressure(&candidate_q) - joint_position_pressure(&baseline_q);
            let joint_velocity = velocity_pressure(&candidate_v) - velocity_pressure(&baseline_v);
            let effort = soft_upper_pressure(
                tube.candidate_actuator_effort_utilization,
                config.actuator_effort_soft_utilization,
            ) - soft_upper_pressure(
                tube.baseline_actuator_effort_utilization,
                config.actuator_effort_soft_utilization,
            );
            let raw_headroom = minimum_headroom(&baseline_q) - minimum_headroom(&candidate_q);
            let components = [
                tilt,
                angular_rate,
                joint_position,
                joint_velocity,
                effort,
                raw_headroom,
            ];
            for (component, value) in components.into_iter().enumerate() {
                assert!(value >= bound.component_lower[component] - 1.0e-12);
                assert!(value <= bound.component_upper[component] + 1.0e-12);
            }
            let aggregate = config.tilt_weight * tilt
                + config.angular_rate_weight * angular_rate
                + config.joint_position_weight * joint_position
                + config.joint_velocity_weight * joint_velocity
                + config.actuator_effort_weight * effort;
            assert!(aggregate >= bound.aggregate_lower - 1.0e-12);
            assert!(aggregate <= bound.aggregate_upper + 1.0e-12);
        }
    }

    #[test]
    fn paired_state_tube_rejects_an_inverted_late_delta() {
        let zero = [0.0; 3];
        let inverted = [0.0, 0.0, 0.1];
        let tube = TerminalImpactPairedStateTube {
            available: true,
            baseline_tilt_lower_rad: [0.0; 2],
            baseline_tilt_upper_rad: [0.0; 2],
            candidate_tilt_delta_lower_rad: [0.0; 2],
            candidate_tilt_delta_upper_rad: [0.0; 2],
            baseline_angular_rate_lower_rad_s: [0.0; 2],
            baseline_angular_rate_upper_rad_s: [0.0; 2],
            candidate_angular_rate_delta_lower_rad_s: [0.0; 2],
            candidate_angular_rate_delta_upper_rad_s: [0.0; 2],
            baseline_joint_position_lower_rad: &zero,
            baseline_joint_position_upper_rad: &zero,
            candidate_joint_position_delta_lower_rad: &inverted,
            candidate_joint_position_delta_upper_rad: &zero,
            baseline_joint_velocity_lower_rad_s: &zero,
            baseline_joint_velocity_upper_rad_s: &zero,
            candidate_joint_velocity_delta_lower_rad_s: &zero,
            candidate_joint_velocity_delta_upper_rad_s: &zero,
            joint_position_limit_lower_rad: &LOWER,
            joint_position_limit_upper_rad: &UPPER,
            joint_velocity_limit_rad_s: &V_LIMIT,
            baseline_actuator_effort_utilization: 0.0,
            candidate_actuator_effort_utilization: 0.0,
        };
        assert_eq!(
            bound_terminal_impact_paired_state_delta(tube, TerminalImpactConfig::default()),
            Err(TerminalImpactError::InvalidState)
        );
    }

    #[test]
    fn velocity_box_rejects_inverted_coordinates() {
        let joint = [0.0; 3];
        let box_state = TerminalImpactVelocityBoxState {
            root_clearance_m: 0.20,
            root_vertical_velocity_lower_m_s: 0.2,
            root_vertical_velocity_upper_m_s: -0.2,
            root_tilt_rad: [0.0; 2],
            root_angular_rate_lower_rad_s: [0.0; 2],
            root_angular_rate_upper_rad_s: [0.0; 2],
            joint_position_rad: &Q,
            joint_velocity_lower_rad_s: &joint,
            joint_velocity_upper_rad_s: &joint,
            joint_position_lower_rad: &LOWER,
            joint_position_upper_rad: &UPPER,
            joint_velocity_limit_rad_s: &V_LIMIT,
        };
        assert_eq!(
            score_terminal_impact_velocity_box_upper(
                box_state,
                candidate([0.0; 2], &joint),
                TerminalImpactConfig::default(),
            ),
            Err(TerminalImpactError::InvalidState)
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
