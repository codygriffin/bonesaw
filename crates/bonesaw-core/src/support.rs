use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{math::Vec3, signal::VectorJet};

/// Contact-side state owned by the controller rather than inferred from solve
/// status codes after the fact.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum SupportPhase {
    #[default]
    Swing = 0,
    Precontact = 1,
    TouchdownNormal = 2,
    Locked = 3,
    NormalFallback = 4,
}

impl SupportPhase {
    pub fn is_contact(self) -> bool {
        matches!(
            self,
            Self::TouchdownNormal | Self::Locked | Self::NormalFallback
        )
    }

    pub fn is_normal_only(self) -> bool {
        matches!(self, Self::TouchdownNormal | Self::NormalFallback)
    }

    pub fn is_planned_transition(self) -> bool {
        matches!(self, Self::Precontact | Self::TouchdownNormal)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SupportTransitionConfig {
    pub minimum_touchdown_ticks: usize,
    pub maximum_lock_tangential_speed_mps: f64,
    /// Maximum material-point distance from the planned landing anchor before
    /// a requested touchdown may become a physical contact.
    pub maximum_touchdown_position_error_m: f64,
    pub maximum_touchdown_tangential_speed_mps: f64,
    pub maximum_touchdown_normal_speed_mps: f64,
}

impl Default for SupportTransitionConfig {
    fn default() -> Self {
        Self {
            minimum_touchdown_ticks: 4,
            maximum_lock_tangential_speed_mps: 0.05,
            maximum_touchdown_position_error_m: 0.025,
            maximum_touchdown_tangential_speed_mps: 0.20,
            maximum_touchdown_normal_speed_mps: 0.20,
        }
    }
}

impl SupportTransitionConfig {
    pub fn validate(self) -> Result<Self, SupportTransitionError> {
        if self.minimum_touchdown_ticks == 0
            || !self.maximum_lock_tangential_speed_mps.is_finite()
            || self.maximum_lock_tangential_speed_mps < 0.0
            || !self.maximum_touchdown_position_error_m.is_finite()
            || self.maximum_touchdown_position_error_m < 0.0
            || !self.maximum_touchdown_tangential_speed_mps.is_finite()
            || self.maximum_touchdown_tangential_speed_mps < 0.0
            || !self.maximum_touchdown_normal_speed_mps.is_finite()
            || self.maximum_touchdown_normal_speed_mps < 0.0
        {
            return Err(SupportTransitionError::Configuration);
        }
        Ok(self)
    }

    /// Decide whether a planned landing is close and slow enough to admit as
    /// contact. A motion schedule requests contact; measured material-point
    /// state is the authority that establishes it.
    pub fn accepts_touchdown(
        self,
        position_error_m: f64,
        tangential_speed_mps: f64,
        normal_speed_mps: f64,
    ) -> Result<bool, SupportTransitionError> {
        self.validate()?;
        if !position_error_m.is_finite()
            || position_error_m < 0.0
            || !tangential_speed_mps.is_finite()
            || tangential_speed_mps < 0.0
            || !normal_speed_mps.is_finite()
            || normal_speed_mps < 0.0
        {
            return Err(SupportTransitionError::TouchdownObservation);
        }
        Ok(position_error_m <= self.maximum_touchdown_position_error_m
            && tangential_speed_mps <= self.maximum_touchdown_tangential_speed_mps
            && normal_speed_mps <= self.maximum_touchdown_normal_speed_mps)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
pub struct SupportTransitionState {
    pub phase: SupportPhase,
    pub phase_ticks: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportTransitionObservation {
    pub entered_contact: bool,
    pub phase_changed: bool,
}

impl SupportTransitionState {
    pub fn advance(
        &mut self,
        contact_requested: bool,
        precontact_requested: bool,
        tangential_speed_mps: Option<f64>,
        config: SupportTransitionConfig,
    ) -> Result<SupportTransitionObservation, SupportTransitionError> {
        let config = config.validate()?;
        if let Some(speed) = tangential_speed_mps
            && (!speed.is_finite() || speed < 0.0)
        {
            return Err(SupportTransitionError::TangentialSpeed);
        }

        let previous = self.phase;
        if !contact_requested {
            if precontact_requested {
                self.set_phase(SupportPhase::Precontact);
            } else {
                self.clear();
            }
        } else {
            match self.phase {
                SupportPhase::Swing | SupportPhase::Precontact => {
                    self.set_phase(SupportPhase::TouchdownNormal);
                }
                SupportPhase::TouchdownNormal => {
                    self.phase_ticks = self.phase_ticks.saturating_add(1);
                    if self.phase_ticks >= config.minimum_touchdown_ticks
                        && tangential_speed_mps
                            .is_some_and(|speed| speed <= config.maximum_lock_tangential_speed_mps)
                    {
                        self.set_phase(SupportPhase::Locked);
                    }
                }
                SupportPhase::Locked | SupportPhase::NormalFallback => {
                    self.phase_ticks = self.phase_ticks.saturating_add(1);
                }
            }
        }
        Ok(SupportTransitionObservation {
            entered_contact: !previous.is_contact() && self.phase.is_contact(),
            phase_changed: previous != self.phase,
        })
    }

    pub fn mark_normal_fallback(&mut self) {
        self.set_phase(SupportPhase::NormalFallback);
    }

    /// Seed a contact that is already established at the beginning of a
    /// trace; startup stance is not a newly detected touchdown.
    pub fn initialize_locked(&mut self) {
        self.phase = SupportPhase::Locked;
        self.phase_ticks = 1;
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

    fn set_phase(&mut self, phase: SupportPhase) {
        if self.phase == phase {
            self.phase_ticks = self.phase_ticks.saturating_add(1);
        } else {
            self.phase = phase;
            self.phase_ticks = 1;
        }
    }
}

#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum SupportTransitionError {
    #[error("support transition configuration is invalid")]
    Configuration,
    #[error("support transition tangential speed is invalid")]
    TangentialSpeed,
    #[error("precontact boundary data is non-finite or has an invalid horizon")]
    PrecontactBoundary,
    #[error("touchdown admission observation is non-finite or negative")]
    TouchdownObservation,
}

/// Initial acceleration of the cubic that joins the measured point state to a
/// touchdown position/velocity over `time_to_touchdown_seconds`.
///
/// Re-evaluating this boundary law every tick gives a deterministic landing
/// shaper without hidden filter memory. Acceleration is norm-clamped so the
/// requested task cannot exceed its declared physical envelope.
pub fn cubic_precontact_acceleration(
    current_position_world: Vec3,
    current_velocity_world: Vec3,
    touchdown_position_world: Vec3,
    touchdown_velocity_world: Vec3,
    time_to_touchdown_seconds: f64,
    maximum_acceleration: f64,
) -> Result<Vec3, SupportTransitionError> {
    if !current_position_world
        .iter()
        .chain(current_velocity_world.iter())
        .chain(touchdown_position_world.iter())
        .chain(touchdown_velocity_world.iter())
        .all(|value| value.is_finite())
        || !time_to_touchdown_seconds.is_finite()
        || time_to_touchdown_seconds <= 0.0
        || !maximum_acceleration.is_finite()
        || maximum_acceleration <= 0.0
    {
        return Err(SupportTransitionError::PrecontactBoundary);
    }
    let horizon = time_to_touchdown_seconds;
    let displacement = touchdown_position_world - current_position_world;
    let acceleration = 6.0 * displacement / (horizon * horizon)
        - (4.0 * current_velocity_world + 2.0 * touchdown_velocity_world) / horizon;
    let norm = acceleration.norm();
    Ok(if norm > maximum_acceleration {
        acceleration * (maximum_acceleration / norm)
    } else {
        acceleration
    })
}

/// Bounds for capture-aware horizontal landing adjustment.
///
/// The policy never changes authored landing height. It can only move the
/// sole-center anchor inside both a disk around the authored endpoint and the
/// future root's spherical leg-reach envelope. The final slew bound makes a
/// changing capture observation continuous at the control rate.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CaptureLandingRetargetConfig {
    /// Signed DCM margin at or above which the authored landing is retained.
    pub activation_margin_m: f64,
    /// Signed DCM margin at or below which the full offset budget is available.
    pub full_scale_margin_m: f64,
    pub maximum_authored_offset_m: f64,
    pub maximum_root_to_landing_reach_m: f64,
    pub maximum_anchor_speed_mps: f64,
}

impl Default for CaptureLandingRetargetConfig {
    fn default() -> Self {
        Self {
            activation_margin_m: 0.0,
            full_scale_margin_m: -0.10,
            maximum_authored_offset_m: 0.12,
            maximum_root_to_landing_reach_m: 0.90,
            maximum_anchor_speed_mps: 0.50,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CaptureLandingRetargetOutput {
    pub desired_anchor_world: Vec3,
    pub applied_anchor_world: Vec3,
    pub capture_scale: f64,
    pub authored_offset_m: f64,
    pub root_to_landing_reach_m: f64,
    pub authored_offset_was_limited: bool,
    pub reach_envelope_was_feasible: bool,
    pub reach_was_limited: bool,
    pub slew_was_limited: bool,
}

/// Bounds for causally retiming an authored contact phase from measured
/// touchdown viability.
///
/// The phase rate is dimensionless: `1` advances the authored reference by one
/// source tick per controller tick, while `0` holds the complete coupled
/// reference. Engagement may be immediate; recovery is normally slower so a
/// noisy observation cannot snap the trajectory back to nominal cadence.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TouchdownPhaseRetimingConfig {
    pub minimum_phase_rate: f64,
    pub maximum_phase_rate: f64,
    pub maximum_phase_rate_increase_per_tick: f64,
    pub maximum_phase_rate_decrease_per_tick: f64,
    /// Time retained beyond the analytic braking estimate.
    pub guard_time_seconds: f64,
}

impl Default for TouchdownPhaseRetimingConfig {
    fn default() -> Self {
        Self {
            minimum_phase_rate: 0.0,
            maximum_phase_rate: 1.0,
            maximum_phase_rate_increase_per_tick: 0.01,
            maximum_phase_rate_decrease_per_tick: 1.0,
            guard_time_seconds: 0.02,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TouchdownPhaseRetimingOutput {
    pub target_phase_rate: f64,
    pub nominal_time_to_touchdown_seconds: f64,
    pub required_time_seconds: f64,
    pub position_time_seconds: f64,
    pub tangential_time_seconds: f64,
    pub normal_time_seconds: f64,
    pub position_limited: bool,
    pub tangential_limited: bool,
    pub normal_limited: bool,
    pub held_at_contact_edge: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TouchdownPhaseRetimingInput {
    pub source_ticks_to_touchdown: f64,
    pub position_error_m: f64,
    pub tangential_speed_mps: f64,
    pub normal_speed_mps: f64,
    pub dt_seconds: f64,
    pub maximum_acceleration_mps2: f64,
}

/// Compute the fastest authored phase rate consistent with the measured
/// landing state and the unchanged touchdown-admission envelope.
///
/// This policy is deliberately scalar and stateless. The caller owns the
/// phase cursor and applied rate, applies the result to every coupled target
/// jet and contact label, and records both as explicit controller state.
pub fn touchdown_phase_retiming(
    input: TouchdownPhaseRetimingInput,
    touchdown: SupportTransitionConfig,
    config: TouchdownPhaseRetimingConfig,
) -> Option<TouchdownPhaseRetimingOutput> {
    let TouchdownPhaseRetimingInput {
        source_ticks_to_touchdown,
        position_error_m,
        tangential_speed_mps,
        normal_speed_mps,
        dt_seconds,
        maximum_acceleration_mps2,
    } = input;
    if !source_ticks_to_touchdown.is_finite()
        || source_ticks_to_touchdown < 0.0
        || !position_error_m.is_finite()
        || position_error_m < 0.0
        || !tangential_speed_mps.is_finite()
        || tangential_speed_mps < 0.0
        || !normal_speed_mps.is_finite()
        || normal_speed_mps < 0.0
        || !dt_seconds.is_finite()
        || dt_seconds <= 0.0
        || !maximum_acceleration_mps2.is_finite()
        || maximum_acceleration_mps2 <= 0.0
        || touchdown.validate().is_err()
        || !config.minimum_phase_rate.is_finite()
        || !config.maximum_phase_rate.is_finite()
        || config.minimum_phase_rate < 0.0
        || config.maximum_phase_rate <= 0.0
        || config.minimum_phase_rate > config.maximum_phase_rate
        || !config.maximum_phase_rate_increase_per_tick.is_finite()
        || config.maximum_phase_rate_increase_per_tick < 0.0
        || !config.maximum_phase_rate_decrease_per_tick.is_finite()
        || config.maximum_phase_rate_decrease_per_tick < 0.0
        || !config.guard_time_seconds.is_finite()
        || config.guard_time_seconds < 0.0
    {
        return None;
    }

    let position_excess =
        (position_error_m - touchdown.maximum_touchdown_position_error_m).max(0.0);
    let tangential_excess =
        (tangential_speed_mps - touchdown.maximum_touchdown_tangential_speed_mps).max(0.0);
    let normal_excess = (normal_speed_mps - touchdown.maximum_touchdown_normal_speed_mps).max(0.0);
    // A conservative scalar envelope: allow time to cancel the observed patch
    // speed and a rest-to-rest acceleration interval for the excess distance.
    // Direction is intentionally not guessed from speed magnitudes.
    let position_time_seconds = tangential_speed_mps / maximum_acceleration_mps2
        + 2.0 * (position_excess / maximum_acceleration_mps2).sqrt();
    let tangential_time_seconds = tangential_excess / maximum_acceleration_mps2;
    let normal_time_seconds = normal_excess / maximum_acceleration_mps2;
    let viability_satisfied =
        position_excess == 0.0 && tangential_excess == 0.0 && normal_excess == 0.0;
    let braking_time_seconds = position_time_seconds
        .max(tangential_time_seconds)
        .max(normal_time_seconds);
    let required_time_seconds = if viability_satisfied {
        0.0
    } else {
        braking_time_seconds + config.guard_time_seconds
    };
    let nominal_time_to_touchdown_seconds = source_ticks_to_touchdown * dt_seconds;
    let held_at_contact_edge = source_ticks_to_touchdown == 0.0 && !viability_satisfied;
    let unconstrained_rate = if viability_satisfied {
        config.maximum_phase_rate
    } else if required_time_seconds > 0.0 {
        nominal_time_to_touchdown_seconds / required_time_seconds
    } else {
        config.maximum_phase_rate
    };
    let target_phase_rate =
        unconstrained_rate.clamp(config.minimum_phase_rate, config.maximum_phase_rate);

    Some(TouchdownPhaseRetimingOutput {
        target_phase_rate,
        nominal_time_to_touchdown_seconds,
        required_time_seconds,
        position_time_seconds,
        tangential_time_seconds,
        normal_time_seconds,
        position_limited: position_time_seconds >= tangential_time_seconds
            && position_time_seconds >= normal_time_seconds
            && position_excess > 0.0,
        tangential_limited: tangential_time_seconds > position_time_seconds
            && tangential_time_seconds >= normal_time_seconds
            && tangential_excess > 0.0,
        normal_limited: normal_time_seconds > position_time_seconds
            && normal_time_seconds > tangential_time_seconds
            && normal_excess > 0.0,
        held_at_contact_edge,
    })
}

/// Apply bounded, non-overshooting phase-rate recovery/engagement.
pub fn slew_touchdown_phase_rate(
    current_phase_rate: f64,
    target_phase_rate: f64,
    config: TouchdownPhaseRetimingConfig,
) -> Option<f64> {
    if !current_phase_rate.is_finite()
        || !target_phase_rate.is_finite()
        || current_phase_rate < config.minimum_phase_rate
        || current_phase_rate > config.maximum_phase_rate
        || target_phase_rate < config.minimum_phase_rate
        || target_phase_rate > config.maximum_phase_rate
        || !config.maximum_phase_rate_increase_per_tick.is_finite()
        || config.maximum_phase_rate_increase_per_tick < 0.0
        || !config.maximum_phase_rate_decrease_per_tick.is_finite()
        || config.maximum_phase_rate_decrease_per_tick < 0.0
    {
        return None;
    }
    let delta = target_phase_rate - current_phase_rate;
    Some(if delta >= 0.0 {
        current_phase_rate + delta.min(config.maximum_phase_rate_increase_per_tick)
    } else {
        current_phase_rate + delta.max(-config.maximum_phase_rate_decrease_per_tick)
    })
}

/// Convert measured signed DCM support margin into a coupled reference rate.
///
/// The reference holds at or below `hold_margin_m`, advances nominally at or
/// above `full_rate_margin_m`, and uses a C1 cubic transition between them.
/// This is a pure policy signal: the caller applies the same rate to every
/// coupled reference consumer and owns any slew/state semantics.
pub fn support_margin_phase_rate(
    dcm_support_margin_m: f64,
    hold_margin_m: f64,
    full_rate_margin_m: f64,
) -> Option<f64> {
    if !dcm_support_margin_m.is_finite()
        || !hold_margin_m.is_finite()
        || !full_rate_margin_m.is_finite()
        || hold_margin_m >= full_rate_margin_m
    {
        return None;
    }
    let phase = ((dcm_support_margin_m - hold_margin_m) / (full_rate_margin_m - hold_margin_m))
        .clamp(0.0, 1.0);
    Some(phase * phase * (3.0 - 2.0 * phase))
}

/// Interpolate a vector jet between adjacent authored samples without heap
/// traffic. Endpoint velocity and acceleration are expressed per second.
pub fn sample_quintic_vector_jet(
    start: VectorJet,
    end: VectorJet,
    duration_seconds: f64,
    phase: f64,
) -> Option<VectorJet> {
    if !start
        .value
        .iter()
        .chain(start.velocity.iter())
        .chain(start.acceleration.iter())
        .chain(end.value.iter())
        .chain(end.velocity.iter())
        .chain(end.acceleration.iter())
        .all(|value| value.is_finite())
        || !duration_seconds.is_finite()
        || duration_seconds <= 0.0
        || !phase.is_finite()
        || !(0.0..=1.0).contains(&phase)
    {
        return None;
    }
    if phase == 0.0 {
        return Some(start);
    }
    if phase == 1.0 {
        return Some(end);
    }
    let duration_squared = duration_seconds * duration_seconds;
    let c0 = start.value;
    let c1 = start.velocity * duration_seconds;
    let c2 = start.acceleration * (0.5 * duration_squared);
    let position_residual = end.value - (c0 + c1 + c2);
    let velocity_residual = end.velocity * duration_seconds - (c1 + 2.0 * c2);
    let acceleration_residual = end.acceleration * duration_squared - 2.0 * c2;
    let c3 = 10.0 * position_residual - 4.0 * velocity_residual + 0.5 * acceleration_residual;
    let c4 = -15.0 * position_residual + 7.0 * velocity_residual - acceleration_residual;
    let c5 = 6.0 * position_residual - 3.0 * velocity_residual + 0.5 * acceleration_residual;
    let value = ((((c5 * phase + c4) * phase + c3) * phase + c2) * phase + c1) * phase + c0;
    let velocity =
        ((((5.0 * c5 * phase + 4.0 * c4) * phase + 3.0 * c3) * phase + 2.0 * c2) * phase + c1)
            / duration_seconds;
    let acceleration = (((20.0 * c5 * phase + 12.0 * c4) * phase + 6.0 * c3) * phase + 2.0 * c2)
        / duration_squared;
    Some(VectorJet {
        value,
        velocity,
        acceleration,
    })
}

/// Apply the chain rule for a dimensionless authored phase rate.
pub fn time_warp_vector_jet(
    authored: VectorJet,
    phase_rate: f64,
    phase_acceleration_per_second: f64,
) -> Option<VectorJet> {
    if !authored
        .value
        .iter()
        .chain(authored.velocity.iter())
        .chain(authored.acceleration.iter())
        .all(|value| value.is_finite())
        || !phase_rate.is_finite()
        || phase_rate < 0.0
        || !phase_acceleration_per_second.is_finite()
    {
        return None;
    }
    Some(VectorJet {
        value: authored.value,
        velocity: authored.velocity * phase_rate,
        acceleration: authored.acceleration * phase_rate.powi(2)
            + authored.velocity * phase_acceleration_per_second,
    })
}

fn smoothstep01(value: f64) -> f64 {
    let value = value.clamp(0.0, 1.0);
    value * value * (3.0 - 2.0 * value)
}

/// Retarget one precontact sole-center anchor toward the measured DCM.
///
/// This is a pure, allocation-free policy primitive. The caller owns the
/// previous applied anchor as explicit controller state and decides which
/// measured phase is allowed to invoke it.
pub fn capture_landing_retarget(
    authored_anchor_world: Vec3,
    current_anchor_world: Vec3,
    future_root_world: Vec3,
    measured_dcm_world: Vec3,
    dcm_support_margin_m: f64,
    dt_seconds: f64,
    config: CaptureLandingRetargetConfig,
) -> Option<CaptureLandingRetargetOutput> {
    if !authored_anchor_world
        .iter()
        .chain(current_anchor_world.iter())
        .chain(future_root_world.iter())
        .chain(measured_dcm_world.iter())
        .all(|value| value.is_finite())
        || !dcm_support_margin_m.is_finite()
        || !dt_seconds.is_finite()
        || dt_seconds <= 0.0
        || !config.activation_margin_m.is_finite()
        || !config.full_scale_margin_m.is_finite()
        || config.full_scale_margin_m >= config.activation_margin_m
        || !config.maximum_authored_offset_m.is_finite()
        || config.maximum_authored_offset_m < 0.0
        || !config.maximum_root_to_landing_reach_m.is_finite()
        || config.maximum_root_to_landing_reach_m <= 0.0
        || !config.maximum_anchor_speed_mps.is_finite()
        || config.maximum_anchor_speed_mps <= 0.0
    {
        return None;
    }

    let capture_phase = (config.activation_margin_m - dcm_support_margin_m)
        / (config.activation_margin_m - config.full_scale_margin_m);
    let capture_scale = smoothstep01(capture_phase);
    let capture_delta = Vec3::new(
        measured_dcm_world.x - authored_anchor_world.x,
        measured_dcm_world.y - authored_anchor_world.y,
        0.0,
    );
    let capture_distance = capture_delta.norm();
    let available_offset = config.maximum_authored_offset_m * capture_scale;
    let authored_offset_was_limited = capture_distance > available_offset;
    let mut desired_distance = available_offset.min(capture_distance);
    let authored_root_delta = authored_anchor_world - future_root_world;
    let authored_reach = authored_root_delta.norm();
    let reach_envelope_was_feasible = authored_reach <= config.maximum_root_to_landing_reach_m;
    let mut reach_was_limited = !reach_envelope_was_feasible;
    if reach_envelope_was_feasible && capture_distance > 0.0 {
        let direction = capture_delta / capture_distance;
        let horizontal_root_delta = Vec3::new(authored_root_delta.x, authored_root_delta.y, 0.0);
        let horizontal_reach_squared =
            config.maximum_root_to_landing_reach_m.powi(2) - authored_root_delta.z.powi(2);
        let along = horizontal_root_delta.dot(&direction);
        let reach_exit_distance = -along
            + (along * along + horizontal_reach_squared - horizontal_root_delta.norm_squared())
                .max(0.0)
                .sqrt();
        if desired_distance > reach_exit_distance {
            desired_distance = reach_exit_distance;
            reach_was_limited = true;
        }
    } else if !reach_envelope_was_feasible {
        desired_distance = 0.0;
    }
    let desired_anchor_world = if capture_distance > 0.0 {
        authored_anchor_world + capture_delta * (desired_distance / capture_distance)
    } else {
        authored_anchor_world
    };

    let mut applied_anchor_world = desired_anchor_world;
    let slew_delta = Vec3::new(
        desired_anchor_world.x - current_anchor_world.x,
        desired_anchor_world.y - current_anchor_world.y,
        0.0,
    );
    let slew_distance = slew_delta.norm();
    let maximum_step = config.maximum_anchor_speed_mps * dt_seconds;
    let slew_was_limited = slew_distance > maximum_step;
    if slew_was_limited {
        applied_anchor_world = current_anchor_world + slew_delta * (maximum_step / slew_distance);
        applied_anchor_world.z = authored_anchor_world.z;
    }

    Some(CaptureLandingRetargetOutput {
        desired_anchor_world,
        applied_anchor_world,
        capture_scale,
        authored_offset_m: (applied_anchor_world - authored_anchor_world)
            .fixed_rows::<2>(0)
            .norm(),
        root_to_landing_reach_m: (applied_anchor_world - future_root_world).norm(),
        authored_offset_was_limited,
        reach_envelope_was_feasible,
        reach_was_limited,
        slew_was_limited,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planned_touchdown_waits_for_time_and_tangential_viability() {
        let config = SupportTransitionConfig::default();
        let mut state = SupportTransitionState::default();
        state.advance(false, true, None, config).unwrap();
        assert_eq!(state.phase, SupportPhase::Precontact);
        let observation = state.advance(true, false, None, config).unwrap();
        assert!(observation.entered_contact);
        assert_eq!(state.phase, SupportPhase::TouchdownNormal);
        for _ in 0..3 {
            state.advance(true, false, Some(0.08), config).unwrap();
        }
        assert_eq!(state.phase, SupportPhase::TouchdownNormal);
        state.advance(true, false, Some(0.049), config).unwrap();
        assert_eq!(state.phase, SupportPhase::Locked);
        assert!(!state.phase.is_normal_only());
    }

    #[test]
    fn normal_fallback_is_distinct_from_planned_touchdown() {
        let mut state = SupportTransitionState::default();
        state
            .advance(true, false, None, SupportTransitionConfig::default())
            .unwrap();
        state.mark_normal_fallback();
        assert_eq!(state.phase, SupportPhase::NormalFallback);
        assert!(state.phase.is_normal_only());
        assert!(!state.phase.is_planned_transition());
        state
            .advance(false, false, None, SupportTransitionConfig::default())
            .unwrap();
        assert_eq!(state, SupportTransitionState::default());
    }

    #[test]
    fn touchdown_admission_requires_both_proximity_and_low_velocity() {
        let config = SupportTransitionConfig::default();
        assert!(config.accepts_touchdown(0.024, 0.19, 0.19).unwrap());
        assert!(!config.accepts_touchdown(0.026, 0.19, 0.19).unwrap());
        assert!(!config.accepts_touchdown(0.024, 0.21, 0.19).unwrap());
        assert!(!config.accepts_touchdown(0.024, 0.19, 0.21).unwrap());
    }

    #[test]
    fn cubic_precontact_boundary_reaches_position_and_velocity() {
        let position = Vec3::new(0.2, -0.1, 0.4);
        let velocity = Vec3::new(0.7, -0.2, -0.4);
        let touchdown = Vec3::new(0.35, -0.05, 0.0);
        let touchdown_velocity = Vec3::zeros();
        let horizon = 0.25;
        let acceleration = cubic_precontact_acceleration(
            position,
            velocity,
            touchdown,
            touchdown_velocity,
            horizon,
            1.0e6,
        )
        .unwrap();
        let jerk =
            2.0 * (touchdown_velocity - velocity - acceleration * horizon) / (horizon * horizon);
        let final_position = position
            + velocity * horizon
            + 0.5 * acceleration * horizon * horizon
            + jerk * horizon.powi(3) / 6.0;
        let final_velocity = velocity + acceleration * horizon + 0.5 * jerk * horizon * horizon;
        assert!((final_position - touchdown).norm() <= 1e-12);
        assert!((final_velocity - touchdown_velocity).norm() <= 1e-12);
    }

    #[test]
    fn capture_landing_retarget_is_bounded_reachable_and_rate_limited() {
        let authored = Vec3::new(0.30, 0.10, 0.0);
        let root = Vec3::new(0.0, 0.0, 0.60);
        let output = capture_landing_retarget(
            authored,
            authored,
            root,
            Vec3::new(2.0, 1.0, 0.6),
            -0.20,
            0.01,
            CaptureLandingRetargetConfig {
                maximum_root_to_landing_reach_m: 0.70,
                maximum_anchor_speed_mps: 0.20,
                ..CaptureLandingRetargetConfig::default()
            },
        )
        .unwrap();
        assert_eq!(output.capture_scale, 1.0);
        assert!(output.authored_offset_was_limited);
        assert!(output.reach_envelope_was_feasible);
        assert!(output.reach_was_limited);
        assert!(output.slew_was_limited);
        assert!((output.applied_anchor_world - authored).norm() <= 0.002 + 1e-12);
        assert_eq!(output.applied_anchor_world.z, authored.z);
        assert!(output.root_to_landing_reach_m <= 0.70 + 1e-12);
    }

    #[test]
    fn capture_landing_retarget_retains_authored_anchor_inside_support() {
        let authored = Vec3::new(0.25, -0.12, 0.0);
        let output = capture_landing_retarget(
            authored,
            authored,
            Vec3::new(0.0, 0.0, 0.6),
            Vec3::new(1.0, 1.0, 0.6),
            0.01,
            0.005,
            CaptureLandingRetargetConfig::default(),
        )
        .unwrap();
        assert_eq!(output.capture_scale, 0.0);
        assert_eq!(output.desired_anchor_world, authored);
        assert_eq!(output.applied_anchor_world, authored);
    }

    #[test]
    fn capture_landing_retarget_reports_unreachable_authored_geometry_without_failure() {
        let authored = Vec3::new(0.25, 0.0, 0.0);
        let output = capture_landing_retarget(
            authored,
            authored,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            -0.20,
            0.005,
            CaptureLandingRetargetConfig::default(),
        )
        .unwrap();
        assert!(!output.reach_envelope_was_feasible);
        assert!(output.reach_was_limited);
        assert_eq!(output.applied_anchor_world, authored);
    }

    #[test]
    fn touchdown_phase_retiming_preserves_nominal_rate_when_viable() {
        let output = touchdown_phase_retiming(
            TouchdownPhaseRetimingInput {
                source_ticks_to_touchdown: 0.0,
                position_error_m: 0.02,
                tangential_speed_mps: 0.19,
                normal_speed_mps: 0.19,
                dt_seconds: 0.005,
                maximum_acceleration_mps2: 25.0,
            },
            SupportTransitionConfig::default(),
            TouchdownPhaseRetimingConfig::default(),
        )
        .unwrap();
        assert_eq!(output.target_phase_rate, 1.0);
        assert_eq!(output.required_time_seconds, 0.0);
        assert!(!output.held_at_contact_edge);
    }

    #[test]
    fn touchdown_phase_retiming_holds_an_unsafe_contact_edge() {
        let output = touchdown_phase_retiming(
            TouchdownPhaseRetimingInput {
                source_ticks_to_touchdown: 0.0,
                position_error_m: 0.20,
                tangential_speed_mps: 1.0,
                normal_speed_mps: 0.1,
                dt_seconds: 0.005,
                maximum_acceleration_mps2: 25.0,
            },
            SupportTransitionConfig::default(),
            TouchdownPhaseRetimingConfig::default(),
        )
        .unwrap();
        assert_eq!(output.target_phase_rate, 0.0);
        assert!(output.required_time_seconds > 0.0);
        assert!(output.position_limited);
        assert!(output.held_at_contact_edge);
    }

    #[test]
    fn touchdown_phase_retiming_scales_available_source_time() {
        let config = TouchdownPhaseRetimingConfig {
            guard_time_seconds: 0.0,
            ..TouchdownPhaseRetimingConfig::default()
        };
        let output = touchdown_phase_retiming(
            TouchdownPhaseRetimingInput {
                source_ticks_to_touchdown: 20.0,
                position_error_m: 0.425,
                tangential_speed_mps: 0.0,
                normal_speed_mps: 0.0,
                dt_seconds: 0.005,
                maximum_acceleration_mps2: 25.0,
            },
            SupportTransitionConfig::default(),
            config,
        )
        .unwrap();
        // Excess distance is 0.4 m, so the rest-to-rest estimate is 0.253 s.
        assert!((output.position_time_seconds - 0.25298221281347033).abs() <= 1e-12);
        assert!((output.target_phase_rate - 0.39528470752104744).abs() <= 1e-12);
    }

    #[test]
    fn touchdown_phase_rate_slew_is_bounded_and_non_overshooting() {
        let config = TouchdownPhaseRetimingConfig {
            maximum_phase_rate_increase_per_tick: 0.05,
            maximum_phase_rate_decrease_per_tick: 0.25,
            ..TouchdownPhaseRetimingConfig::default()
        };
        assert_eq!(slew_touchdown_phase_rate(1.0, 0.0, config), Some(0.75));
        assert_eq!(slew_touchdown_phase_rate(0.2, 1.0, config), Some(0.25));
        assert_eq!(slew_touchdown_phase_rate(0.98, 1.0, config), Some(1.0));
    }

    #[test]
    fn support_margin_phase_rate_holds_then_recovers_smoothly() {
        assert_eq!(support_margin_phase_rate(-0.01, 0.0, 0.02), Some(0.0));
        assert_eq!(support_margin_phase_rate(0.0, 0.0, 0.02), Some(0.0));
        assert_eq!(support_margin_phase_rate(0.01, 0.0, 0.02), Some(0.5));
        assert_eq!(support_margin_phase_rate(0.02, 0.0, 0.02), Some(1.0));
        assert_eq!(support_margin_phase_rate(0.03, 0.0, 0.02), Some(1.0));
        assert!(support_margin_phase_rate(0.0, 0.02, 0.0).is_none());
    }

    #[test]
    fn quintic_vector_jet_preserves_both_boundary_jets() {
        let start = VectorJet {
            value: Vec3::new(0.2, -0.1, 0.5),
            velocity: Vec3::new(0.7, 0.1, -0.2),
            acceleration: Vec3::new(-0.3, 0.4, 0.5),
        };
        let end = VectorJet {
            value: Vec3::new(0.8, 0.2, 0.1),
            velocity: Vec3::new(-0.2, 0.3, 0.0),
            acceleration: Vec3::new(0.1, -0.2, 0.4),
        };
        assert_eq!(
            sample_quintic_vector_jet(start, end, 0.005, 0.0),
            Some(start)
        );
        let sampled_end = sample_quintic_vector_jet(start, end, 0.005, 1.0).unwrap();
        assert!((sampled_end.value - end.value).norm() <= 1e-12);
        assert!((sampled_end.velocity - end.velocity).norm() <= 1e-9);
        assert!((sampled_end.acceleration - end.acceleration).norm() <= 1e-6);
    }

    #[test]
    fn time_warp_vector_jet_applies_the_full_chain_rule() {
        let authored = VectorJet {
            value: Vec3::new(1.0, 2.0, 3.0),
            velocity: Vec3::new(2.0, -1.0, 0.5),
            acceleration: Vec3::new(4.0, 2.0, -3.0),
        };
        let warped = time_warp_vector_jet(authored, 0.5, -2.0).unwrap();
        assert_eq!(warped.value, authored.value);
        assert_eq!(warped.velocity, authored.velocity * 0.5);
        assert_eq!(
            warped.acceleration,
            authored.acceleration * 0.25 + authored.velocity * -2.0
        );
    }
}
