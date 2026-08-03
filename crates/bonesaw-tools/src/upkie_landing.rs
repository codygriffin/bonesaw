//! Measured, phase-aware landing/reload state for the Upkie example.
//!
//! This module is deliberately a request/phase boundary rather than a hidden
//! torque controller.  The caller supplies the exact physics-window contact,
//! the debounced observation masks, measured normal loads, and wheel geometry.
//! Rust owns the transition state, the force-backed reacquisition witness, the
//! precontact boundary law, and the bounded free-leg request.  The returned
//! acceleration and contact modes still have to pass the ordinary floating WBC.

use bonesaw_core::{
    ContactReacquisitionConfig, ContactReacquisitionOutput, ContactReacquisitionState,
    SupportPhase, SupportTransitionConfig, SupportTransitionState, Vec3,
    cubic_precontact_acceleration, step_contact_reacquisition,
};
use nalgebra::Matrix3;

use crate::{
    UpkieSingleSupportReacquisitionConfig, UpkieSingleSupportReacquisitionOutput,
    UpkieSingleSupportReacquisitionState, step_upkie_single_support_reacquisition,
};

/// Fixed number of diagnostic values exposed by the PyO3 adapter.
pub const UPKIE_MEASURED_LANDING_DIAGNOSTIC_WIDTH: usize = 25;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieMeasuredLandingConfig {
    pub request: UpkieSingleSupportReacquisitionConfig,
    pub transition: SupportTransitionConfig,
    pub reacquisition: ContactReacquisitionConfig,
    pub target_support_mask: u8,
    pub precontact_horizon_seconds: f64,
    /// Consecutive 50 Hz observations required before a one-wheel physics
    /// window is treated as a landing phase.  Precontact is intentionally
    /// immediate: a single measured loss is enough to protect the free leg,
    /// while contact-mode promotion remains force-backed and debounced.
    pub minimum_precontact_ticks: u8,
    /// Continuous request envelope limits.  These are deliberately inert at
    /// their defaults so the R309/R312 phase boundary remains unchanged until
    /// an evaluation profile opts into the landing/moment safety envelope.
    pub maximum_request_tilt_rad: f64,
    pub maximum_request_horizontal_speed_m_s: f64,
    pub minimum_request_height_m: f64,
    pub request_height_blend_m: f64,
}

impl Default for UpkieMeasuredLandingConfig {
    fn default() -> Self {
        Self {
            request: UpkieSingleSupportReacquisitionConfig::default(),
            transition: SupportTransitionConfig::default(),
            reacquisition: ContactReacquisitionConfig::default(),
            target_support_mask: 3,
            precontact_horizon_seconds: 0.20,
            minimum_precontact_ticks: 1,
            maximum_request_tilt_rad: std::f64::consts::PI,
            maximum_request_horizontal_speed_m_s: 1.0e6,
            minimum_request_height_m: 0.0,
            request_height_blend_m: 0.10,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UpkieMeasuredLandingState {
    pub initialized: bool,
    pub observer_started: bool,
    pub tick_sequence: u64,
    pub single_support_ticks: u8,
    pub transitions: [SupportTransitionState; 2],
    pub needs_relock: [bool; 2],
    pub request: UpkieSingleSupportReacquisitionState,
    pub reacquisition: ContactReacquisitionState<2>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieMeasuredLandingOutput {
    pub physics_support_mask: u8,
    pub observed_support_mask: u8,
    pub target_support_mask: u8,
    pub precontact_active: bool,
    pub touchdown_normal_active: bool,
    pub locked_count: u8,
    pub phases: [SupportPhase; 2],
    /// `0` disabled, `2` NormalPoint, `3` RollingWheel.
    pub contact_modes: [u8; 2],
    pub precontact_acceleration_world_m_s2: Vec3,
    pub precontact_acceleration_was_limited: bool,
    pub request: UpkieSingleSupportReacquisitionOutput,
    pub reacquisition: ContactReacquisitionOutput<2>,
}

fn valid_config(config: UpkieMeasuredLandingConfig) -> bool {
    config.target_support_mask == 3
        && config.precontact_horizon_seconds.is_finite()
        && config.precontact_horizon_seconds > 0.0
        && config.minimum_precontact_ticks > 0
        && config.maximum_request_tilt_rad.is_finite()
        && config.maximum_request_tilt_rad > 0.0
        && config.maximum_request_horizontal_speed_m_s.is_finite()
        && config.maximum_request_horizontal_speed_m_s > 0.0
        && config.minimum_request_height_m.is_finite()
        && config.minimum_request_height_m >= 0.0
        && config.request_height_blend_m.is_finite()
        && config.request_height_blend_m > 0.0
        && config.transition.validate().is_ok()
        && config.request.target_wheel_height_m.is_finite()
        && config.reacquisition.required_samples > 0
}

fn smoothstep_unit(value: f64) -> f64 {
    let phase = value.clamp(0.0, 1.0);
    phase * phase * (3.0 - 2.0 * phase)
}

/// Apply a continuous, fail-closed envelope to a free-leg landing request.
///
/// The phase observer and force-backed contact witness remain unchanged.  The
/// envelope only scales the already-bounded request as the measured root gets
/// too tilted, too fast, or too close to the ground.  This prevents a stale
/// landing action from spending the remaining support margin during a moment
/// rejection event while retaining a smooth authority signal for telemetry.
pub fn apply_upkie_measured_landing_request_envelope(
    root_height_m: f64,
    root_tilt_rad: f64,
    root_horizontal_speed_m_s: f64,
    config: UpkieMeasuredLandingConfig,
    request: &mut UpkieSingleSupportReacquisitionOutput,
    joint_acceleration_out: &mut [f64; 6],
) -> Option<f64> {
    if !root_height_m.is_finite()
        || !root_tilt_rad.is_finite()
        || !root_horizontal_speed_m_s.is_finite()
        || root_height_m < 0.0
        || root_tilt_rad < 0.0
        || root_horizontal_speed_m_s < 0.0
        || !valid_config(config)
    {
        return None;
    }
    if !request.active {
        return Some(0.0);
    }
    let tilt_pressure = root_tilt_rad / config.maximum_request_tilt_rad;
    let speed_pressure = root_horizontal_speed_m_s / config.maximum_request_horizontal_speed_m_s;
    let height_pressure = (config.minimum_request_height_m + config.request_height_blend_m
        - root_height_m)
        / config.request_height_blend_m;
    let scale = (1.0 - smoothstep_unit(tilt_pressure))
        .min(1.0 - smoothstep_unit(speed_pressure))
        .min(1.0 - smoothstep_unit(height_pressure))
        .clamp(0.0, 1.0);
    request.authority *= scale;
    request.commanded_vertical_acceleration_m_s2 *= scale;
    request.active = scale > 0.0 && request.authority > 0.0;
    for acceleration in joint_acceleration_out.iter_mut() {
        *acceleration *= scale;
    }
    Some(scale)
}

fn mask_is_binary(mask: u8) -> bool {
    mask <= 3
}

fn damped_joint_acceleration(
    jacobian: &[[f64; 6]; 3],
    desired_acceleration: Vec3,
    damping: f64,
    authority: f64,
    maximum_joint_acceleration: f64,
    output: &mut [f64; 6],
) -> bool {
    if !damping.is_finite()
        || damping <= 0.0
        || !authority.is_finite()
        || !(0.0..=1.0).contains(&authority)
        || !maximum_joint_acceleration.is_finite()
        || maximum_joint_acceleration <= 0.0
        || jacobian.iter().flatten().any(|value| !value.is_finite())
        || desired_acceleration.iter().any(|value| !value.is_finite())
    {
        return false;
    }
    let mut gram = Matrix3::zeros();
    for row in 0..3 {
        for column in 0..3 {
            gram[(row, column)] = (0..6)
                .map(|joint| jacobian[row][joint] * jacobian[column][joint])
                .sum::<f64>();
        }
    }
    gram[(0, 0)] += damping * damping;
    gram[(1, 1)] += damping * damping;
    gram[(2, 2)] += damping * damping;
    let Some(inverse) = gram.try_inverse() else {
        return false;
    };
    let solved = inverse * desired_acceleration;
    for joint in 0..6 {
        let value = authority
            * (0..3)
                .map(|axis| jacobian[axis][joint] * solved[axis])
                .sum::<f64>();
        output[joint] = value.clamp(-maximum_joint_acceleration, maximum_joint_acceleration);
    }
    true
}

/// Advance one measured Upkie landing/reload tick.
///
/// `physics_support_mask` is the exact contact mask at the beginning of the
/// 250 Hz physics window.  `observed_support_mask` is the 50 Hz debounced
/// authority mask.  Keeping both makes the precontact boundary explicit and
/// prevents a delayed WBC observation from being mistaken for future support.
#[allow(clippy::too_many_arguments)]
pub fn step_upkie_measured_landing(
    timestep_seconds: f64,
    tick_sequence: u64,
    physics_observation_exact: bool,
    physics_support_mask: u8,
    observed_support_mask: u8,
    raw_contact: [bool; 2],
    stable_contact: [bool; 2],
    hard_contact: [bool; 2],
    normal_load_n: [f64; 2],
    wheel_positions_world: [Vec3; 2],
    wheel_velocities_world: [Vec3; 2],
    wheel_joint_jacobians: [[[f64; 6]; 3]; 2],
    joint_velocity: &[f64; 6],
    config: UpkieMeasuredLandingConfig,
    state: &mut UpkieMeasuredLandingState,
    joint_acceleration_out: &mut [f64; 6],
) -> Option<UpkieMeasuredLandingOutput> {
    if !timestep_seconds.is_finite()
        || timestep_seconds <= 0.0
        || !mask_is_binary(physics_support_mask)
        || !mask_is_binary(observed_support_mask)
        || !valid_config(config)
        || state.tick_sequence >= tick_sequence && state.initialized
        || normal_load_n
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        || wheel_positions_world
            .iter()
            .chain(wheel_velocities_world.iter())
            .flat_map(|vector| vector.iter())
            .any(|value| !value.is_finite())
        || wheel_joint_jacobians
            .iter()
            .flatten()
            .flatten()
            .any(|value| !value.is_finite())
        || joint_velocity.iter().any(|value| !value.is_finite())
    {
        return None;
    }
    joint_acceleration_out.fill(0.0);
    state.tick_sequence = tick_sequence;

    if physics_observation_exact && matches!(physics_support_mask, 1 | 2) {
        state.single_support_ticks = state.single_support_ticks.saturating_add(1);
    } else {
        state.single_support_ticks = 0;
    }
    let landing_window_active = state.single_support_ticks >= config.minimum_precontact_ticks;

    // Do not turn contact-observation startup/debounce into a false loss and
    // reacquisition.  The observer target becomes bilateral only after the
    // measured/debounced authority has established the initial stance.
    if !state.observer_started
        && physics_observation_exact
        && physics_support_mask == 3
        && observed_support_mask == 3
    {
        state.observer_started = true;
    }
    let target = if state.observer_started {
        [true, true]
    } else {
        [false, false]
    };
    let reacquisition = step_contact_reacquisition(
        tick_sequence,
        physics_observation_exact,
        target,
        raw_contact,
        stable_contact,
        hard_contact,
        normal_load_n,
        config.reacquisition,
        &mut state.reacquisition,
    );

    if !state.initialized && physics_observation_exact && physics_support_mask == 3 {
        for transition in &mut state.transitions {
            transition.initialize_locked();
        }
        state.initialized = true;
    }
    let mut precontact_active = false;
    let mut touchdown_normal_active = false;
    let mut locked_count = 0u8;
    for index in 0..2 {
        let bit = 1u8 << index;
        let contact = physics_observation_exact && physics_support_mask & bit != 0;
        if !contact && config.target_support_mask & bit != 0 {
            state.needs_relock[index] = true;
        }
        let tangential_speed =
            contact.then(|| wheel_velocities_world[index].fixed_rows::<2>(0).norm());
        let transition = state.transitions[index]
            .advance(
                contact,
                physics_observation_exact
                    && landing_window_active
                    && config.target_support_mask & bit != 0
                    && !contact,
                tangential_speed,
                config.transition,
            )
            .ok()?;
        let phase = state.transitions[index].phase;
        precontact_active |= phase == SupportPhase::Precontact;
        touchdown_normal_active |= phase == SupportPhase::TouchdownNormal;
        if phase == SupportPhase::Locked {
            locked_count = locked_count.saturating_add(1);
        }
        if phase == SupportPhase::Locked && reacquisition.qualified {
            state.needs_relock[index] = false;
        }
        let _ = transition;
    }

    let lost_support_index = match physics_support_mask {
        1 => 1,
        2 => 0,
        _ => 0,
    };
    let mut vertical_jacobian = [0.0; 6];
    for joint in 0..6 {
        vertical_jacobian[joint] = wheel_joint_jacobians[lost_support_index][2][joint];
    }
    let request_support_mask = if landing_window_active {
        physics_support_mask
    } else {
        3
    };
    let request = step_upkie_single_support_reacquisition(
        timestep_seconds,
        physics_observation_exact,
        request_support_mask,
        wheel_positions_world[lost_support_index].z,
        wheel_velocities_world[lost_support_index].z,
        &vertical_jacobian,
        joint_velocity,
        config.request,
        &mut state.request,
        joint_acceleration_out,
    )?;

    let mut precontact_acceleration = Vec3::zeros();
    let mut precontact_acceleration_was_limited = false;
    if request.active && precontact_active {
        let current = wheel_positions_world[lost_support_index];
        let target_position = Vec3::new(current.x, current.y, config.request.target_wheel_height_m);
        precontact_acceleration = cubic_precontact_acceleration(
            current,
            wheel_velocities_world[lost_support_index],
            target_position,
            Vec3::zeros(),
            config.precontact_horizon_seconds,
            config.request.maximum_vertical_acceleration_m_s2,
        )
        .ok()?;
        precontact_acceleration_was_limited = precontact_acceleration.norm()
            >= config.request.maximum_vertical_acceleration_m_s2 - 1.0e-12;
        let mut shaped = [0.0; 6];
        if damped_joint_acceleration(
            &wheel_joint_jacobians[lost_support_index],
            precontact_acceleration,
            config.request.jacobian_damping,
            request.authority,
            config.request.maximum_joint_acceleration_rad_s2,
            &mut shaped,
        ) {
            joint_acceleration_out.copy_from_slice(&shaped);
        }
    }

    let mut contact_modes = [0u8; 2];
    for index in 0..2 {
        let bit = 1u8 << index;
        if config.target_support_mask & bit == 0 {
            continue;
        }
        contact_modes[index] = if state.needs_relock[index] || observed_support_mask & bit == 0 {
            2
        } else {
            3
        };
    }
    Some(UpkieMeasuredLandingOutput {
        physics_support_mask,
        observed_support_mask,
        target_support_mask: config.target_support_mask,
        precontact_active,
        touchdown_normal_active,
        locked_count,
        phases: [state.transitions[0].phase, state.transitions[1].phase],
        contact_modes,
        precontact_acceleration_world_m_s2: precontact_acceleration,
        precontact_acceleration_was_limited,
        request,
        reacquisition,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs() -> ([Vec3; 2], [Vec3; 2], [[[f64; 6]; 3]; 2], [f64; 6]) {
        (
            [Vec3::new(0.18, 0.14, 0.05), Vec3::new(0.18, -0.14, 0.05)],
            [Vec3::zeros(), Vec3::zeros()],
            [[[0.0; 6]; 3]; 2],
            [0.0; 6],
        )
    }

    #[test]
    fn loss_enters_precontact_and_new_contact_stays_normal_until_qualified() {
        let (positions, velocities, mut jacobians, joint_velocity) = inputs();
        jacobians[1][2][2] = 1.0;
        let mut state = UpkieMeasuredLandingState::default();
        let config = UpkieMeasuredLandingConfig::default();
        let mut qdd = [0.0; 6];
        let baseline = step_upkie_measured_landing(
            0.02,
            1,
            true,
            3,
            3,
            [true, true],
            [true, true],
            [true, true],
            [25.0, 25.0],
            positions,
            velocities,
            jacobians,
            &joint_velocity,
            config,
            &mut state,
            &mut qdd,
        )
        .unwrap();
        assert_eq!(baseline.contact_modes, [3, 3]);
        let chatter = step_upkie_measured_landing(
            0.02,
            2,
            true,
            1,
            3,
            [true, false],
            [true, true],
            [true, true],
            [25.0, 0.0],
            positions,
            velocities,
            jacobians,
            &joint_velocity,
            config,
            &mut state,
            &mut qdd,
        )
        .unwrap();
        assert!(chatter.precontact_active);
        assert!(chatter.request.active);
        assert_eq!(chatter.contact_modes, [3, 2]);
        let loss = step_upkie_measured_landing(
            0.02,
            3,
            true,
            1,
            3,
            [true, false],
            [true, true],
            [true, true],
            [25.0, 0.0],
            positions,
            velocities,
            jacobians,
            &joint_velocity,
            config,
            &mut state,
            &mut qdd,
        )
        .unwrap();
        assert!(loss.precontact_active);
        let active_loss = step_upkie_measured_landing(
            0.02,
            4,
            true,
            1,
            3,
            [true, false],
            [true, true],
            [true, true],
            [25.0, 0.0],
            positions,
            velocities,
            jacobians,
            &joint_velocity,
            config,
            &mut state,
            &mut qdd,
        )
        .unwrap();
        assert!(active_loss.precontact_active);
        assert!(active_loss.request.active);
        assert_eq!(active_loss.contact_modes[1], 2);
        for tick in 5..8 {
            let output = step_upkie_measured_landing(
                0.02,
                tick,
                true,
                3,
                3,
                [true, true],
                [true, true],
                [true, true],
                [20.0, 20.0],
                positions,
                velocities,
                jacobians,
                &joint_velocity,
                config,
                &mut state,
                &mut qdd,
            )
            .unwrap();
            if tick < 7 {
                assert_eq!(output.contact_modes[1], 2);
            }
        }
        let qualified = step_upkie_measured_landing(
            0.02,
            8,
            true,
            3,
            3,
            [true, true],
            [true, true],
            [true, true],
            [20.0, 20.0],
            positions,
            velocities,
            jacobians,
            &joint_velocity,
            config,
            &mut state,
            &mut qdd,
        )
        .unwrap();
        assert!(qualified.reacquisition.qualified);
        assert_eq!(qualified.contact_modes, [3, 3]);
    }
    #[test]
    fn flight_clears_stale_rolling_modes() {
        let (positions, velocities, jacobians, joint_velocity) = inputs();
        let mut state = UpkieMeasuredLandingState::default();
        let config = UpkieMeasuredLandingConfig::default();
        let mut qdd = [0.0; 6];
        let _ = step_upkie_measured_landing(
            0.02,
            1,
            true,
            3,
            3,
            [true, true],
            [true, true],
            [true, true],
            [25.0, 25.0],
            positions,
            velocities,
            jacobians,
            &joint_velocity,
            config,
            &mut state,
            &mut qdd,
        )
        .unwrap();
        let flight = step_upkie_measured_landing(
            0.02,
            2,
            true,
            0,
            0,
            [false, false],
            [false, false],
            [false, false],
            [0.0, 0.0],
            positions,
            velocities,
            jacobians,
            &joint_velocity,
            config,
            &mut state,
            &mut qdd,
        )
        .unwrap();
        assert_eq!(flight.contact_modes, [2, 2]);
    }

    #[test]
    fn request_envelope_scales_continuously_and_fails_closed_at_limits() {
        let mut config = UpkieMeasuredLandingConfig::default();
        config.maximum_request_tilt_rad = 0.5;
        config.maximum_request_horizontal_speed_m_s = 2.0;
        config.minimum_request_height_m = 0.30;
        config.request_height_blend_m = 0.10;
        let mut request_state = UpkieSingleSupportReacquisitionState::default();
        let mut qdd = [1.0; 6];
        let mut request = step_upkie_single_support_reacquisition(
            0.02,
            true,
            1,
            0.04,
            0.0,
            &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            &[0.0; 6],
            config.request,
            &mut request_state,
            &mut qdd,
        )
        .unwrap();
        let full = apply_upkie_measured_landing_request_envelope(
            0.41,
            0.0,
            0.0,
            config,
            &mut request,
            &mut qdd,
        )
        .unwrap();
        assert_eq!(full, 1.0);
        let before = request.authority;
        let partial = apply_upkie_measured_landing_request_envelope(
            0.39,
            0.25,
            0.5,
            config,
            &mut request,
            &mut qdd,
        )
        .unwrap();
        assert!(partial > 0.0 && partial < 1.0);
        assert!(request.authority < before);
        let zero = apply_upkie_measured_landing_request_envelope(
            0.39,
            0.5,
            0.0,
            config,
            &mut request,
            &mut qdd,
        )
        .unwrap();
        assert_eq!(zero, 0.0);
        assert!(!request.active);
        assert_eq!(qdd, [0.0; 6]);
    }
}
