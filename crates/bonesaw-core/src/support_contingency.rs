//! Support-conditioned, low-energy contingency request generation.
//!
//! This module deliberately stops at request generation.  It does not claim
//! that the floating system can realize the request: callers must submit the
//! emitted acceleration to the ordinary dynamics/contact/effort admission
//! path for the current observed support set.

use crate::math::Vec3;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum SupportContingencyMode {
    #[default]
    DoubleSupport = 0,
    SingleSupport = 1,
    Flight = 2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SupportContingencyConfig {
    pub gravity_mps2: f64,
    pub double_support_position_gain_per_s2: f64,
    pub single_support_position_gain_per_s2: f64,
    pub supported_linear_damping_per_s: f64,
    pub attitude_stiffness_per_s2: f64,
    pub angular_damping_per_s: f64,
    pub joint_damping_per_s: f64,
    pub maximum_horizontal_acceleration_m_s2: f64,
    pub maximum_vertical_braking_acceleration_m_s2: f64,
    pub maximum_angular_acceleration_rad_s2: f64,
    pub maximum_joint_acceleration_rad_s2: f64,
}

impl Default for SupportContingencyConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: 9.81,
            double_support_position_gain_per_s2: 4.0,
            single_support_position_gain_per_s2: 8.0,
            supported_linear_damping_per_s: 4.0,
            attitude_stiffness_per_s2: 40.0,
            angular_damping_per_s: 10.0,
            joint_damping_per_s: 8.0,
            maximum_horizontal_acceleration_m_s2: 8.0,
            maximum_vertical_braking_acceleration_m_s2: 20.0,
            maximum_angular_acceleration_rad_s2: 80.0,
            maximum_joint_acceleration_rad_s2: 120.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SupportContingencyEvidence {
    pub mode: SupportContingencyMode,
    pub support_mask: u8,
    pub active_support_count: u8,
    pub center_of_mass_error_world_m: Vec3,
    pub requested_horizontal_acceleration_world_m_s2: Vec3,
    pub requested_tilt_rotation_vector_world_rad: Vec3,
    pub ballistic_vertical_acceleration: bool,
    pub horizontal_acceleration_was_saturated: bool,
    pub vertical_acceleration_was_saturated: bool,
    pub angular_acceleration_was_saturated: bool,
    pub joint_acceleration_was_saturated: bool,
}

fn finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

/// Write one allocation-free contingency request from current support
/// evidence. `support_mask` is limited to the two Upkie support sites for this
/// first CPU checkpoint.  For flight, the support centroid is ignored and the
/// linear request is exactly ballistic gravity; no fictitious ground force is
/// requested.
#[allow(clippy::too_many_arguments)]
pub fn write_support_contingency_request(
    support_mask: u8,
    center_of_mass_position_world_m: Vec3,
    active_support_centroid_world_m: Vec3,
    root_rotation_vector_world_rad: Vec3,
    root_twist_world: &[f64; 6],
    joint_velocity: &[f64],
    config: SupportContingencyConfig,
    root_angular_acceleration_out: &mut [f64; 3],
    root_linear_acceleration_out: &mut [f64; 3],
    joint_acceleration_out: &mut [f64],
) -> Option<SupportContingencyEvidence> {
    let config_values = [
        config.gravity_mps2,
        config.double_support_position_gain_per_s2,
        config.single_support_position_gain_per_s2,
        config.supported_linear_damping_per_s,
        config.attitude_stiffness_per_s2,
        config.angular_damping_per_s,
        config.joint_damping_per_s,
        config.maximum_horizontal_acceleration_m_s2,
        config.maximum_vertical_braking_acceleration_m_s2,
        config.maximum_angular_acceleration_rad_s2,
        config.maximum_joint_acceleration_rad_s2,
    ];
    if support_mask > 3
        || joint_velocity.len() != joint_acceleration_out.len()
        || config_values
            .into_iter()
            .any(|value| !finite_positive(value))
        || center_of_mass_position_world_m
            .iter()
            .chain(active_support_centroid_world_m.iter())
            .chain(root_rotation_vector_world_rad.iter())
            .chain(root_twist_world.iter())
            .chain(joint_velocity.iter())
            .any(|value| !value.is_finite())
    {
        return None;
    }

    let active_support_count = support_mask.count_ones() as u8;
    let mode = match active_support_count {
        2 => SupportContingencyMode::DoubleSupport,
        1 => SupportContingencyMode::SingleSupport,
        _ => SupportContingencyMode::Flight,
    };
    let mut center_of_mass_error = Vec3::zeros();
    let mut horizontal = Vec3::zeros();
    let mut linear = [0.0; 3];
    let horizontal_gain = match mode {
        SupportContingencyMode::DoubleSupport => config.double_support_position_gain_per_s2,
        SupportContingencyMode::SingleSupport => config.single_support_position_gain_per_s2,
        SupportContingencyMode::Flight => 0.0,
    };
    let mut horizontal_saturated = false;
    let mut vertical_saturated = false;
    if mode == SupportContingencyMode::Flight {
        linear[2] = -config.gravity_mps2;
    } else {
        center_of_mass_error = active_support_centroid_world_m - center_of_mass_position_world_m;
        center_of_mass_error.z = 0.0;
        horizontal.x = horizontal_gain * center_of_mass_error.x
            - config.supported_linear_damping_per_s * root_twist_world[3];
        horizontal.y = horizontal_gain * center_of_mass_error.y
            - config.supported_linear_damping_per_s * root_twist_world[4];
        let norm = horizontal.xy().norm();
        if norm > config.maximum_horizontal_acceleration_m_s2 {
            horizontal *= config.maximum_horizontal_acceleration_m_s2 / norm;
            horizontal_saturated = true;
        }
        linear[0] = horizontal.x;
        linear[1] = horizontal.y;
        let unsaturated_vertical = -config.supported_linear_damping_per_s * root_twist_world[5];
        linear[2] = unsaturated_vertical.clamp(
            -config.maximum_vertical_braking_acceleration_m_s2,
            config.maximum_vertical_braking_acceleration_m_s2,
        );
        vertical_saturated = linear[2] != unsaturated_vertical;
    }

    // Small-angle inertial tilt consistent with the horizontal acceleration:
    // +ay requests +roll, while +ax requests -pitch.
    let requested_tilt = if mode == SupportContingencyMode::Flight {
        Vec3::zeros()
    } else {
        Vec3::new(
            horizontal.y / config.gravity_mps2,
            -horizontal.x / config.gravity_mps2,
            0.0,
        )
    };
    let mut angular = [0.0; 3];
    let mut angular_saturated = false;
    for axis in 0..3 {
        let requested = config.attitude_stiffness_per_s2
            * (requested_tilt[axis] - root_rotation_vector_world_rad[axis])
            - config.angular_damping_per_s * root_twist_world[axis];
        angular[axis] = requested.clamp(
            -config.maximum_angular_acceleration_rad_s2,
            config.maximum_angular_acceleration_rad_s2,
        );
        angular_saturated |= angular[axis] != requested;
    }
    let mut joint_saturated = false;
    for (output, velocity) in joint_acceleration_out.iter_mut().zip(joint_velocity) {
        let requested = -config.joint_damping_per_s * velocity;
        *output = requested.clamp(
            -config.maximum_joint_acceleration_rad_s2,
            config.maximum_joint_acceleration_rad_s2,
        );
        joint_saturated |= *output != requested;
    }
    root_angular_acceleration_out.copy_from_slice(&angular);
    root_linear_acceleration_out.copy_from_slice(&linear);
    Some(SupportContingencyEvidence {
        mode,
        support_mask,
        active_support_count,
        center_of_mass_error_world_m: center_of_mass_error,
        requested_horizontal_acceleration_world_m_s2: horizontal,
        requested_tilt_rotation_vector_world_rad: requested_tilt,
        ballistic_vertical_acceleration: mode == SupportContingencyMode::Flight,
        horizontal_acceleration_was_saturated: horizontal_saturated,
        vertical_acceleration_was_saturated: vertical_saturated,
        angular_acceleration_was_saturated: angular_saturated,
        joint_acceleration_was_saturated: joint_saturated,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(mask: u8, support: Vec3) -> (SupportContingencyEvidence, [f64; 3], [f64; 3]) {
        let mut angular = [0.0; 3];
        let mut linear = [0.0; 3];
        let mut joint = [0.0; 2];
        let evidence = write_support_contingency_request(
            mask,
            Vec3::new(0.0, 0.1, 0.5),
            support,
            Vec3::zeros(),
            &[0.0; 6],
            &[1.0, -1.0],
            SupportContingencyConfig::default(),
            &mut angular,
            &mut linear,
            &mut joint,
        )
        .unwrap();
        assert_eq!(joint, [-8.0, 8.0]);
        (evidence, angular, linear)
    }

    #[test]
    fn double_and_single_support_pull_com_toward_observed_support() {
        let (double, _, double_linear) = run(3, Vec3::zeros());
        let (single, single_angular, single_linear) = run(1, Vec3::new(0.0, -0.2, 0.0));
        assert_eq!(double.mode, SupportContingencyMode::DoubleSupport);
        assert_eq!(single.mode, SupportContingencyMode::SingleSupport);
        assert!(double_linear[1] < 0.0);
        assert!(single_linear[1] < double_linear[1]);
        assert!(single_angular[0] < 0.0);
    }

    #[test]
    fn mirrored_single_support_requests_mirrored_action() {
        let (_, left_angular, left_linear) = run(1, Vec3::new(0.0, -0.2, 0.0));
        let mut angular = [0.0; 3];
        let mut linear = [0.0; 3];
        let mut joint = [0.0; 2];
        write_support_contingency_request(
            2,
            Vec3::new(0.0, -0.1, 0.5),
            Vec3::new(0.0, 0.2, 0.0),
            Vec3::zeros(),
            &[0.0; 6],
            &[1.0, -1.0],
            SupportContingencyConfig::default(),
            &mut angular,
            &mut linear,
            &mut joint,
        )
        .unwrap();
        assert_eq!(left_linear[1], -linear[1]);
        assert_eq!(left_angular[0], -angular[0]);
    }

    #[test]
    fn flight_is_ballistic_and_does_not_invent_support_force() {
        let (evidence, angular, linear) = run(0, Vec3::new(99.0, 99.0, 99.0));
        assert_eq!(evidence.mode, SupportContingencyMode::Flight);
        assert_eq!(linear, [0.0, 0.0, -9.81]);
        assert_eq!(angular, [0.0; 3]);
        assert!(evidence.ballistic_vertical_acceleration);
    }

    #[test]
    fn invalid_input_is_atomic() {
        let mut angular = [1.0; 3];
        let mut linear = [2.0; 3];
        let mut joint = [3.0; 2];
        assert!(
            write_support_contingency_request(
                4,
                Vec3::zeros(),
                Vec3::zeros(),
                Vec3::zeros(),
                &[0.0; 6],
                &[0.0; 2],
                SupportContingencyConfig::default(),
                &mut angular,
                &mut linear,
                &mut joint,
            )
            .is_none()
        );
        assert_eq!(angular, [1.0; 3]);
        assert_eq!(linear, [2.0; 3]);
        assert_eq!(joint, [3.0; 2]);
    }
}
