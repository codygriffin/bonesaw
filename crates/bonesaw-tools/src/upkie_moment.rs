//! Bounded, state-local body-moment rejection for the Upkie example.
//!
//! This layer authors one roll-acceleration delta. It does not create contact,
//! change solver rows, or bypass the ordinary floating-WBC admission boundary.

/// Fixed diagnostic width exposed by the Python adapter.
pub const UPKIE_BODY_MOMENT_DIAGNOSTIC_WIDTH: usize = 11;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieBodyMomentRejectionConfig {
    pub activation_release_tilt_rad: f64,
    pub activation_full_tilt_rad: f64,
    pub activation_release_outward_rate_rad_s: f64,
    pub activation_full_outward_rate_rad_s: f64,
    pub restoring_stiffness_per_s2: f64,
    pub outward_damping_per_s: f64,
    pub maximum_roll_acceleration_rad_s2: f64,
}

impl Default for UpkieBodyMomentRejectionConfig {
    fn default() -> Self {
        Self {
            activation_release_tilt_rad: 0.02,
            activation_full_tilt_rad: 0.12,
            activation_release_outward_rate_rad_s: 0.10,
            activation_full_outward_rate_rad_s: 0.80,
            restoring_stiffness_per_s2: 36.0,
            outward_damping_per_s: 14.0,
            maximum_roll_acceleration_rad_s2: 10.5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpkieBodyMomentRejectionOutput {
    pub active: bool,
    pub measured_roll_rad: f64,
    pub measured_roll_rate_rad_s: f64,
    pub outward_direction: f64,
    pub outward_rate_rad_s: f64,
    pub tilt_pressure: f64,
    pub outward_rate_pressure: f64,
    pub authority: f64,
    pub requested_roll_acceleration_rad_s2: f64,
    pub commanded_roll_acceleration_rad_s2: f64,
    pub acceleration_was_saturated: bool,
}

fn smoothstep_unit(value: f64) -> f64 {
    let phase = value.clamp(0.0, 1.0);
    phase * phase * (3.0 - 2.0 * phase)
}

fn valid_config(config: UpkieBodyMomentRejectionConfig) -> bool {
    let values = [
        config.activation_release_tilt_rad,
        config.activation_full_tilt_rad,
        config.activation_release_outward_rate_rad_s,
        config.activation_full_outward_rate_rad_s,
        config.restoring_stiffness_per_s2,
        config.outward_damping_per_s,
        config.maximum_roll_acceleration_rad_s2,
    ];
    values.iter().all(|value| value.is_finite())
        && config.activation_release_tilt_rad >= 0.0
        && config.activation_release_tilt_rad < config.activation_full_tilt_rad
        && config.activation_release_outward_rate_rad_s >= 0.0
        && config.activation_release_outward_rate_rad_s < config.activation_full_outward_rate_rad_s
        && config.restoring_stiffness_per_s2 >= 0.0
        && config.outward_damping_per_s >= 0.0
        && config.maximum_roll_acceleration_rad_s2 > 0.0
}

/// Author one bounded roll-acceleration delta from current measured state.
///
/// Damping is spent only while roll rate points farther away from upright.
/// Restoring stiffness is smoothly activated by either tilt or outward rate.
/// The caller remains responsible for blending the delta into a task and for
/// ordinary floating-WBC admission.
pub fn step_upkie_body_moment_rejection(
    measured_roll_rad: f64,
    measured_roll_rate_rad_s: f64,
    config: UpkieBodyMomentRejectionConfig,
) -> Option<UpkieBodyMomentRejectionOutput> {
    if !measured_roll_rad.is_finite()
        || !measured_roll_rate_rad_s.is_finite()
        || !valid_config(config)
    {
        return None;
    }
    let direction_source = if measured_roll_rad.abs() > 1.0e-9 {
        measured_roll_rad
    } else {
        measured_roll_rate_rad_s
    };
    let outward_direction = if direction_source.is_sign_negative() {
        -1.0
    } else {
        1.0
    };
    let outward_rate = (outward_direction * measured_roll_rate_rad_s).max(0.0);
    let tilt_pressure = smoothstep_unit(
        (measured_roll_rad.abs() - config.activation_release_tilt_rad)
            / (config.activation_full_tilt_rad - config.activation_release_tilt_rad),
    );
    let outward_rate_pressure = smoothstep_unit(
        (outward_rate - config.activation_release_outward_rate_rad_s)
            / (config.activation_full_outward_rate_rad_s
                - config.activation_release_outward_rate_rad_s),
    );
    let authority = tilt_pressure.max(outward_rate_pressure);
    let requested = -config.restoring_stiffness_per_s2 * measured_roll_rad
        - config.outward_damping_per_s * outward_direction * outward_rate;
    let bounded = requested.clamp(
        -config.maximum_roll_acceleration_rad_s2,
        config.maximum_roll_acceleration_rad_s2,
    );
    let commanded = authority * bounded;
    Some(UpkieBodyMomentRejectionOutput {
        active: authority > 0.0 && commanded != 0.0,
        measured_roll_rad,
        measured_roll_rate_rad_s,
        outward_direction,
        outward_rate_rad_s: outward_rate,
        tilt_pressure,
        outward_rate_pressure,
        authority,
        requested_roll_acceleration_rad_s2: requested,
        commanded_roll_acceleration_rad_s2: commanded,
        acceleration_was_saturated: requested != bounded,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mirrored_outward_state_produces_mirrored_bounded_command() {
        let config = UpkieBodyMomentRejectionConfig::default();
        let positive = step_upkie_body_moment_rejection(0.10, 0.50, config).unwrap();
        let negative = step_upkie_body_moment_rejection(-0.10, -0.50, config).unwrap();
        assert!(positive.active && negative.active);
        assert_eq!(positive.authority, negative.authority);
        assert_eq!(positive.outward_rate_rad_s, negative.outward_rate_rad_s);
        assert_eq!(
            positive.commanded_roll_acceleration_rad_s2,
            -negative.commanded_roll_acceleration_rad_s2
        );
        assert!(positive.commanded_roll_acceleration_rad_s2 < 0.0);
        assert!(
            positive.commanded_roll_acceleration_rad_s2.abs()
                <= config.maximum_roll_acceleration_rad_s2
        );
    }

    #[test]
    fn dormant_upright_state_is_exact_zero() {
        let output =
            step_upkie_body_moment_rejection(0.0, 0.0, UpkieBodyMomentRejectionConfig::default())
                .unwrap();
        assert!(!output.active);
        assert_eq!(output.authority, 0.0);
        assert_eq!(output.commanded_roll_acceleration_rad_s2, 0.0);
    }

    #[test]
    fn inward_rate_does_not_spend_damping() {
        let config = UpkieBodyMomentRejectionConfig::default();
        let output = step_upkie_body_moment_rejection(0.10, -0.50, config).unwrap();
        assert_eq!(output.outward_rate_rad_s, 0.0);
        assert_eq!(
            output.requested_roll_acceleration_rad_s2,
            -config.restoring_stiffness_per_s2 * 0.10
        );
    }

    #[test]
    fn invalid_input_fails_closed() {
        assert!(
            step_upkie_body_moment_rejection(
                f64::NAN,
                0.0,
                UpkieBodyMomentRejectionConfig::default(),
            )
            .is_none()
        );
    }
}
