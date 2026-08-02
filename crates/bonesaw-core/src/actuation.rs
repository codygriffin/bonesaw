use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::model::{CompiledModel, JointId};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ActuatorId(pub usize);

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct ActuatorLimits {
    #[serde(with = "crate::model::serde_extended_f64")]
    pub velocity: f64,
    #[serde(with = "crate::model::serde_extended_f64")]
    pub acceleration: f64,
    #[serde(with = "crate::model::serde_extended_f64")]
    pub effort: f64,
    #[serde(with = "crate::model::serde_extended_f64")]
    pub jerk: f64,
}

impl ActuatorLimits {
    pub fn validate(self) -> bool {
        [self.velocity, self.acceleration, self.effort, self.jerk]
            .into_iter()
            .all(|limit| limit > 0.0 && !limit.is_nan())
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct ActuatorResourceModel {
    /// Motor torque constant at the actuator coordinate after transmission.
    pub torque_constant_nm_per_amp: f64,
    pub winding_resistance_ohm: f64,
    pub thermal_resistance_c_per_w: f64,
    pub thermal_time_constant_s: f64,
    pub ambient_temperature_c: f64,
    /// Continuous physical derating begins here. UI warning thresholds remain
    /// external to this model.
    pub derating_start_temperature_c: f64,
    /// Available effort reaches `minimum_effort_fraction` here.
    pub shutdown_temperature_c: f64,
    pub minimum_effort_fraction: f64,
}

impl ActuatorResourceModel {
    pub fn validate(self) -> bool {
        self.torque_constant_nm_per_amp.is_finite()
            && self.torque_constant_nm_per_amp > 0.0
            && self.winding_resistance_ohm.is_finite()
            && self.winding_resistance_ohm > 0.0
            && self.thermal_resistance_c_per_w.is_finite()
            && self.thermal_resistance_c_per_w > 0.0
            && self.thermal_time_constant_s.is_finite()
            && self.thermal_time_constant_s > 0.0
            && self.ambient_temperature_c.is_finite()
            && self.derating_start_temperature_c.is_finite()
            && self.shutdown_temperature_c.is_finite()
            && self.derating_start_temperature_c > self.ambient_temperature_c
            && self.shutdown_temperature_c > self.derating_start_temperature_c
            && self.minimum_effort_fraction.is_finite()
            && (0.0..=1.0).contains(&self.minimum_effort_fraction)
    }

    pub fn effort_scale(self, winding_temperature_c: f64) -> Option<f64> {
        if !self.validate() || !winding_temperature_c.is_finite() {
            return None;
        }
        if winding_temperature_c <= self.derating_start_temperature_c {
            return Some(1.0);
        }
        if winding_temperature_c >= self.shutdown_temperature_c {
            return Some(self.minimum_effort_fraction);
        }
        let phase = (winding_temperature_c - self.derating_start_temperature_c)
            / (self.shutdown_temperature_c - self.derating_start_temperature_c);
        Some(1.0 + phase * (self.minimum_effort_fraction - 1.0))
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ActuatorSpec {
    pub id: ActuatorId,
    pub name: String,
    pub limits: ActuatorLimits,
    /// Absent unless a calibrated electrical/thermal profile was authored.
    pub resource_model: Option<ActuatorResourceModel>,
}

/// Explicit linear actuation map. Generalized velocity is
/// `generalized_from_actuator * actuator_velocity`. The optional reverse map
/// is authored only where generalized commands uniquely map to actuators.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CompiledActuation {
    pub actuators: Vec<ActuatorSpec>,
    pub generalized_from_actuator: DMatrix<f64>,
    pub actuator_from_generalized: Option<DMatrix<f64>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorResourceState {
    pub winding_temperature_c: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorResourceSample {
    pub current_a: f64,
    pub copper_loss_w: f64,
    pub mechanical_power_w: f64,
    /// Ideal DC-side power before drive, switching, and regeneration losses.
    pub ideal_electrical_power_w: f64,
    pub winding_temperature_c: f64,
    pub effort_scale: f64,
    pub derated_effort_limit_nm: f64,
    pub effort_headroom_nm: f64,
    /// None only when the authored physical model declares zero available
    /// effort at or above shutdown.
    pub effort_utilization: Option<f64>,
}

/// Synthetic command-path dynamics used to score whether an admitted WBC
/// effort trace can be followed by a declared actuator response envelope.
/// These parameters are profile data, never inferred from the URDF.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct ActuatorRealizationProfile {
    /// Closed-loop effort bandwidth. Positive infinity is the exact-response
    /// control case.
    pub effort_bandwidth_hz: f64,
    /// Maximum realized effort change. Positive infinity disables slew
    /// limiting without changing the response equation.
    pub maximum_effort_rate_nm_per_s: f64,
}

impl ActuatorRealizationProfile {
    pub fn validate(self) -> bool {
        self.effort_bandwidth_hz > 0.0
            && !self.effort_bandwidth_hz.is_nan()
            && self.maximum_effort_rate_nm_per_s > 0.0
            && !self.maximum_effort_rate_nm_per_s.is_nan()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorRealizationState {
    pub realized_effort_nm: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorRealizationSample {
    pub requested_effort_nm: f64,
    pub available_effort_limit_nm: f64,
    pub limited_target_effort_nm: f64,
    pub realized_effort_nm: f64,
    pub tracking_error_nm: f64,
    pub availability_clipped: bool,
    pub slew_limited: bool,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct WheelCommand {
    pub left_velocity: f64,
    pub right_velocity: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DifferentialDriveMap {
    pub left_wheel: JointId,
    pub right_wheel: JointId,
    pub wheel_radius: f64,
    pub track_width: f64,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum ActuationError {
    #[error("wheel radius and track width must be finite and positive")]
    InvalidGeometry,
    #[error("wheel joint `{0}` does not exist or is fixed")]
    MissingWheel(String),
    #[error("actuation map dimensions do not match the model")]
    Dimension,
    #[error("actuation map or actuator limits contain invalid values")]
    InvalidMap,
    #[error("actuator electrical/thermal profile is invalid")]
    InvalidResourceModel,
    #[error("actuator resource sample contains invalid input")]
    InvalidResourceInput,
    #[error("actuator realization profile is invalid")]
    InvalidRealizationProfile,
    #[error("actuator realization sample contains invalid input")]
    InvalidRealizationInput,
}

impl CompiledActuation {
    /// Canonical URDF fallback: one explicit actuator per generalized
    /// coordinate with an identity transmission. Coupled/passive mechanisms
    /// replace this at authoring time rather than remaining implicit.
    pub fn identity_from_model(model: &CompiledModel) -> Self {
        let mut actuators = Vec::with_capacity(model.dof);
        for coordinate in 0..model.dof {
            let joint = model
                .joints
                .iter()
                .find(|joint| joint.coordinate == Some(coordinate))
                .expect("canonical model owns every generalized coordinate");
            actuators.push(ActuatorSpec {
                id: ActuatorId(coordinate),
                name: joint.name.clone(),
                limits: ActuatorLimits {
                    velocity: joint.limit.velocity,
                    acceleration: f64::INFINITY,
                    effort: joint.limit.effort,
                    jerk: f64::INFINITY,
                },
                resource_model: None,
            });
        }
        Self {
            actuators,
            generalized_from_actuator: DMatrix::identity(model.dof, model.dof),
            actuator_from_generalized: Some(DMatrix::identity(model.dof, model.dof)),
        }
    }

    pub fn validate(&self, model: &CompiledModel) -> Result<(), ActuationError> {
        let actuator_count = self.actuators.len();
        if self.generalized_from_actuator.nrows() != model.dof
            || self.generalized_from_actuator.ncols() != actuator_count
            || self
                .actuator_from_generalized
                .as_ref()
                .is_some_and(|map| map.nrows() != actuator_count || map.ncols() != model.dof)
        {
            return Err(ActuationError::Dimension);
        }
        if self.actuators.iter().enumerate().any(|(index, actuator)| {
            actuator.id.0 != index || actuator.name.is_empty() || !actuator.limits.validate()
        }) {
            return Err(ActuationError::InvalidMap);
        }
        if self.actuators.iter().any(|actuator| {
            actuator
                .resource_model
                .is_some_and(|resource| !resource.validate())
        }) {
            return Err(ActuationError::InvalidResourceModel);
        }
        if self
            .generalized_from_actuator
            .iter()
            .any(|value| !value.is_finite())
            || self
                .actuator_from_generalized
                .as_ref()
                .is_some_and(|map| map.iter().any(|value| !value.is_finite()))
            || (0..actuator_count).any(|actuator| {
                self.generalized_from_actuator
                    .column(actuator)
                    .iter()
                    .all(|value| *value == 0.0)
            })
        {
            return Err(ActuationError::InvalidMap);
        }
        if let Some(reverse) = &self.actuator_from_generalized {
            for row in 0..actuator_count {
                for column in 0..actuator_count {
                    let actual: f64 = (0..model.dof)
                        .map(|coordinate| {
                            reverse[(row, coordinate)]
                                * self.generalized_from_actuator[(coordinate, column)]
                        })
                        .sum();
                    let expected = f64::from(row == column);
                    if (actual - expected).abs() > 1e-10 {
                        return Err(ActuationError::InvalidMap);
                    }
                }
            }
        }
        Ok(())
    }

    pub fn write_generalized_velocity(
        &self,
        actuator_velocity: &DVector<f64>,
        generalized_velocity: &mut DVector<f64>,
    ) -> Result<(), ActuationError> {
        if actuator_velocity.len() != self.actuators.len()
            || generalized_velocity.len() != self.generalized_from_actuator.nrows()
        {
            return Err(ActuationError::Dimension);
        }
        for row in 0..generalized_velocity.len() {
            generalized_velocity[row] = (0..actuator_velocity.len())
                .map(|column| {
                    self.generalized_from_actuator[(row, column)] * actuator_velocity[column]
                })
                .sum();
        }
        Ok(())
    }

    pub fn write_actuator_velocity(
        &self,
        generalized_velocity: &DVector<f64>,
        actuator_velocity: &mut DVector<f64>,
    ) -> Result<(), ActuationError> {
        let reverse = self
            .actuator_from_generalized
            .as_ref()
            .ok_or(ActuationError::InvalidMap)?;
        if generalized_velocity.len() != reverse.ncols()
            || actuator_velocity.len() != reverse.nrows()
        {
            return Err(ActuationError::Dimension);
        }
        for row in 0..actuator_velocity.len() {
            actuator_velocity[row] = (0..generalized_velocity.len())
                .map(|column| reverse[(row, column)] * generalized_velocity[column])
                .sum();
        }
        Ok(())
    }

    /// Power-consistent dual map for `qdot = G * adot`: actuator effort is
    /// `G.transpose() * generalized_effort`.
    pub fn write_actuator_effort(
        &self,
        generalized_effort: &DVector<f64>,
        actuator_effort: &mut DVector<f64>,
    ) -> Result<(), ActuationError> {
        if generalized_effort.len() != self.generalized_from_actuator.nrows()
            || actuator_effort.len() != self.generalized_from_actuator.ncols()
        {
            return Err(ActuationError::Dimension);
        }
        for actuator in 0..actuator_effort.len() {
            actuator_effort[actuator] = (0..generalized_effort.len())
                .map(|coordinate| {
                    self.generalized_from_actuator[(coordinate, actuator)]
                        * generalized_effort[coordinate]
                })
                .sum();
        }
        Ok(())
    }

    /// Returns generalized effort limits when the transmission is independent
    /// diagonal. General coupled actuator polytopes require WBC inequality rows
    /// and are deliberately not approximated as coordinate-wise bounds.
    pub fn independent_generalized_effort_limits(&self) -> Option<Vec<f64>> {
        let dof = self.generalized_from_actuator.nrows();
        if self.actuators.len() != dof || self.generalized_from_actuator.ncols() != dof {
            return None;
        }
        let mut limits = Vec::with_capacity(dof);
        for coordinate in 0..dof {
            if (0..dof).any(|actuator| {
                actuator != coordinate
                    && self.generalized_from_actuator[(coordinate, actuator)] != 0.0
            }) || (0..dof).any(|row| {
                row != coordinate && self.generalized_from_actuator[(row, coordinate)] != 0.0
            }) {
                return None;
            }
            let ratio = self.generalized_from_actuator[(coordinate, coordinate)].abs();
            let actuator_limit = self.actuators[coordinate].limits.effort;
            if !ratio.is_finite()
                || ratio <= 0.0
                || actuator_limit.is_nan()
                || actuator_limit <= 0.0
            {
                return None;
            }
            limits.push(actuator_limit / ratio);
        }
        Some(limits)
    }
}

pub fn step_actuator_resource(
    model: ActuatorResourceModel,
    state: &mut ActuatorResourceState,
    actuator_effort_nm: f64,
    actuator_velocity_rad_s: f64,
    base_effort_limit_nm: f64,
    dt_seconds: f64,
) -> Result<ActuatorResourceSample, ActuationError> {
    if !model.validate() {
        return Err(ActuationError::InvalidResourceModel);
    }
    if !state.winding_temperature_c.is_finite()
        || !actuator_effort_nm.is_finite()
        || !actuator_velocity_rad_s.is_finite()
        || !base_effort_limit_nm.is_finite()
        || base_effort_limit_nm <= 0.0
        || !dt_seconds.is_finite()
        || dt_seconds <= 0.0
    {
        return Err(ActuationError::InvalidResourceInput);
    }
    let current_a = actuator_effort_nm.abs() / model.torque_constant_nm_per_amp;
    let copper_loss_w = current_a * current_a * model.winding_resistance_ohm;
    let mechanical_power_w = actuator_effort_nm * actuator_velocity_rad_s;
    let ideal_electrical_power_w = mechanical_power_w + copper_loss_w;
    let steady_rise_c = copper_loss_w * model.thermal_resistance_c_per_w;
    let decay = (-dt_seconds / model.thermal_time_constant_s).exp();
    let next_temperature = model.ambient_temperature_c
        + steady_rise_c
        + (state.winding_temperature_c - model.ambient_temperature_c - steady_rise_c) * decay;
    state.winding_temperature_c = next_temperature;
    let effort_scale = model
        .effort_scale(next_temperature)
        .ok_or(ActuationError::InvalidResourceModel)?;
    let derated_effort_limit_nm = base_effort_limit_nm * effort_scale;
    let effort_headroom_nm = derated_effort_limit_nm - actuator_effort_nm.abs();
    let effort_utilization = (derated_effort_limit_nm > 0.0)
        .then_some(actuator_effort_nm.abs() / derated_effort_limit_nm);
    Ok(ActuatorResourceSample {
        current_a,
        copper_loss_w,
        mechanical_power_w,
        ideal_electrical_power_w,
        winding_temperature_c: next_temperature,
        effort_scale,
        derated_effort_limit_nm,
        effort_headroom_nm,
        effort_utilization,
    })
}

pub fn step_actuator_realization(
    profile: ActuatorRealizationProfile,
    state: &mut ActuatorRealizationState,
    requested_effort_nm: f64,
    available_effort_limit_nm: f64,
    dt_seconds: f64,
) -> Result<ActuatorRealizationSample, ActuationError> {
    if !profile.validate() {
        return Err(ActuationError::InvalidRealizationProfile);
    }
    if !state.realized_effort_nm.is_finite()
        || !requested_effort_nm.is_finite()
        || available_effort_limit_nm <= 0.0
        || available_effort_limit_nm.is_nan()
        || !dt_seconds.is_finite()
        || dt_seconds <= 0.0
    {
        return Err(ActuationError::InvalidRealizationInput);
    }
    let limited_target_effort_nm =
        requested_effort_nm.clamp(-available_effort_limit_nm, available_effort_limit_nm);
    let availability_clipped = limited_target_effort_nm != requested_effort_nm;
    let bandwidth_effort_nm = if profile.effort_bandwidth_hz.is_infinite() {
        limited_target_effort_nm
    } else {
        let response_fraction =
            1.0 - (-std::f64::consts::TAU * profile.effort_bandwidth_hz * dt_seconds).exp();
        state.realized_effort_nm
            + response_fraction * (limited_target_effort_nm - state.realized_effort_nm)
    };
    let bandwidth_delta = bandwidth_effort_nm - state.realized_effort_nm;
    let maximum_delta = profile.maximum_effort_rate_nm_per_s * dt_seconds;
    let limited_delta = bandwidth_delta.clamp(-maximum_delta, maximum_delta);
    let slew_limited = limited_delta != bandwidth_delta;
    let slew_effort_nm = if profile.maximum_effort_rate_nm_per_s.is_infinite() {
        bandwidth_effort_nm
    } else {
        state.realized_effort_nm + limited_delta
    };
    // Availability is an instantaneous drive constraint. It may tighten more
    // quickly than the declared effort slew and therefore clamps the final
    // command independently of the response state.
    let realized_effort_nm =
        slew_effort_nm.clamp(-available_effort_limit_nm, available_effort_limit_nm);
    state.realized_effort_nm = realized_effort_nm;
    Ok(ActuatorRealizationSample {
        requested_effort_nm,
        available_effort_limit_nm,
        limited_target_effort_nm,
        realized_effort_nm,
        tracking_error_nm: requested_effort_nm - realized_effort_nm,
        availability_clipped,
        slew_limited,
    })
}

impl DifferentialDriveMap {
    pub fn compile(
        model: &CompiledModel,
        left_wheel: &str,
        right_wheel: &str,
        wheel_radius: f64,
        track_width: f64,
    ) -> Result<Self, ActuationError> {
        if !wheel_radius.is_finite()
            || wheel_radius <= 0.0
            || !track_width.is_finite()
            || track_width <= 0.0
        {
            return Err(ActuationError::InvalidGeometry);
        }
        let resolve = |name: &str| {
            model
                .joint_id(name)
                .filter(|joint| model.joints[joint.0].coordinate.is_some())
                .ok_or_else(|| ActuationError::MissingWheel(name.to_owned()))
        };
        Ok(Self {
            left_wheel: resolve(left_wheel)?,
            right_wheel: resolve(right_wheel)?,
            wheel_radius,
            track_width,
        })
    }

    pub fn inverse_kinematics(&self, forward_velocity: f64, yaw_rate: f64) -> WheelCommand {
        let half_track = 0.5 * self.track_width;
        WheelCommand {
            left_velocity: (forward_velocity - half_track * yaw_rate) / self.wheel_radius,
            right_velocity: (forward_velocity + half_track * yaw_rate) / self.wheel_radius,
        }
    }

    pub fn forward_kinematics(&self, command: WheelCommand) -> (f64, f64) {
        let forward = 0.5 * self.wheel_radius * (command.left_velocity + command.right_velocity);
        let yaw_rate =
            self.wheel_radius * (command.right_velocity - command.left_velocity) / self.track_width;
        (forward, yaw_rate)
    }

    pub fn write_generalized_velocity(
        &self,
        model: &CompiledModel,
        command: WheelCommand,
        velocity: &mut nalgebra::DVector<f64>,
    ) {
        let left = model.joints[self.left_wheel.0]
            .coordinate
            .expect("compiled wheel is actuated");
        let right = model.joints[self.right_wheel.0]
            .coordinate
            .expect("compiled wheel is actuated");
        velocity[left] = command.left_velocity;
        velocity[right] = command.right_velocity;
    }
}

#[cfg(test)]
mod tests {
    use crate::urdf::load_urdf;

    use super::*;

    #[test]
    fn differential_drive_round_trip() {
        let source = r#"
        <robot name="wheels">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
          <link name="left"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <link name="right"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <joint name="left_wheel" type="continuous"><parent link="base"/><child link="left"/><axis xyz="0 1 0"/><limit velocity="10" effort="2"/></joint>
          <joint name="right_wheel" type="continuous"><parent link="base"/><child link="right"/><axis xyz="0 1 0"/><limit velocity="10" effort="2"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let mapping =
            DifferentialDriveMap::compile(&model, "left_wheel", "right_wheel", 0.1, 0.4).unwrap();
        let command = mapping.inverse_kinematics(0.8, -0.3);
        let (forward, yaw) = mapping.forward_kinematics(command);
        assert!((forward - 0.8).abs() < 1e-12);
        assert!((yaw + 0.3).abs() < 1e-12);
    }

    #[test]
    fn explicit_identity_actuation_maps_velocity_and_effort_without_allocation() {
        let source = r#"
        <robot name="actuation">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
          <link name="tip"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <joint name="hinge" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="7" effort="12"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let actuation = CompiledActuation::identity_from_model(&model);
        actuation.validate(&model).unwrap();
        assert_eq!(actuation.actuators[0].name, "hinge");
        assert_eq!(actuation.actuators[0].limits.effort, 12.0);
        assert_eq!(
            actuation.independent_generalized_effort_limits(),
            Some(vec![12.0])
        );

        let source = DVector::from_element(1, -3.5);
        let mut mapped = DVector::zeros(1);
        actuation
            .write_generalized_velocity(&source, &mut mapped)
            .unwrap();
        assert_eq!(mapped, source);
        mapped.fill(0.0);
        actuation
            .write_actuator_effort(&source, &mut mapped)
            .unwrap();
        assert_eq!(mapped, source);
    }

    #[test]
    fn exact_lumped_thermal_step_heats_cools_and_derates_continuously() {
        let model = ActuatorResourceModel {
            torque_constant_nm_per_amp: 1.0,
            winding_resistance_ohm: 2.0,
            thermal_resistance_c_per_w: 0.5,
            thermal_time_constant_s: 10.0,
            ambient_temperature_c: 20.0,
            derating_start_temperature_c: 30.0,
            shutdown_temperature_c: 40.0,
            minimum_effort_fraction: 0.0,
        };
        let mut state = ActuatorResourceState {
            winding_temperature_c: 20.0,
        };
        let heated = step_actuator_resource(model, &mut state, 4.0, 3.0, 10.0, 10.0).unwrap();
        let expected = 20.0 + 16.0 * (1.0 - (-1.0_f64).exp());
        assert!((heated.winding_temperature_c - expected).abs() < 1e-12);
        assert_eq!(heated.current_a, 4.0);
        assert_eq!(heated.copper_loss_w, 32.0);
        assert_eq!(heated.mechanical_power_w, 12.0);
        assert_eq!(heated.ideal_electrical_power_w, 44.0);
        assert!(heated.effort_scale < 1.0);
        assert!(heated.effort_utilization.unwrap() > 0.4);

        let before_cooling = state.winding_temperature_c;
        let cooled = step_actuator_resource(model, &mut state, 0.0, 0.0, 10.0, 10.0).unwrap();
        assert!(cooled.winding_temperature_c < before_cooling);
        assert!(cooled.winding_temperature_c > model.ambient_temperature_c);
    }

    #[test]
    fn actuator_realization_has_exact_ideal_and_first_order_controls() {
        let mut ideal_state = ActuatorRealizationState {
            realized_effort_nm: -2.0,
        };
        let ideal = step_actuator_realization(
            ActuatorRealizationProfile {
                effort_bandwidth_hz: f64::INFINITY,
                maximum_effort_rate_nm_per_s: f64::INFINITY,
            },
            &mut ideal_state,
            7.0,
            10.0,
            0.005,
        )
        .unwrap();
        assert_eq!(ideal.realized_effort_nm, 7.0);
        assert_eq!(ideal.tracking_error_nm, 0.0);
        assert!(!ideal.availability_clipped);
        assert!(!ideal.slew_limited);

        let mut lagged_state = ActuatorRealizationState {
            realized_effort_nm: 0.0,
        };
        let lagged = step_actuator_realization(
            ActuatorRealizationProfile {
                effort_bandwidth_hz: 1.0,
                maximum_effort_rate_nm_per_s: f64::INFINITY,
            },
            &mut lagged_state,
            10.0,
            20.0,
            0.1,
        )
        .unwrap();
        let expected = 10.0 * (1.0 - (-0.1 * std::f64::consts::TAU).exp());
        assert!((lagged.realized_effort_nm - expected).abs() < 1e-12);
    }

    #[test]
    fn actuator_realization_separates_slew_from_availability_clipping() {
        let profile = ActuatorRealizationProfile {
            effort_bandwidth_hz: f64::INFINITY,
            maximum_effort_rate_nm_per_s: 5.0,
        };
        let mut slew_state = ActuatorRealizationState {
            realized_effort_nm: 0.0,
        };
        let slew = step_actuator_realization(profile, &mut slew_state, 10.0, 20.0, 0.1).unwrap();
        assert_eq!(slew.realized_effort_nm, 0.5);
        assert!(slew.slew_limited);
        assert!(!slew.availability_clipped);

        let mut availability_state = ActuatorRealizationState {
            realized_effort_nm: 0.0,
        };
        let availability =
            step_actuator_realization(profile, &mut availability_state, 10.0, 0.25, 0.1).unwrap();
        assert_eq!(availability.limited_target_effort_nm, 0.25);
        assert_eq!(availability.realized_effort_nm, 0.25);
        assert!(availability.availability_clipped);
        assert!(!availability.slew_limited);
    }

    #[test]
    fn coupled_transmission_preserves_power_and_round_trips_velocity() {
        let source = r#"
        <robot name="coupled">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
          <link name="middle"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <link name="tip"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <joint name="first" type="continuous"><parent link="base"/><child link="middle"/><axis xyz="0 1 0"/><limit velocity="7" effort="12"/></joint>
          <joint name="second" type="continuous"><parent link="middle"/><child link="tip"/><axis xyz="0 1 0"/><limit velocity="7" effort="12"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let mut actuation = CompiledActuation::identity_from_model(&model);
        actuation.generalized_from_actuator = DMatrix::from_row_slice(2, 2, &[0.5, 0.5, -1.0, 1.0]);
        actuation.actuator_from_generalized =
            Some(DMatrix::from_row_slice(2, 2, &[1.0, -0.5, 1.0, 0.5]));
        actuation.validate(&model).unwrap();
        assert_eq!(actuation.independent_generalized_effort_limits(), None);

        let actuator_velocity = DVector::from_vec(vec![2.0, 4.0]);
        let mut generalized_velocity = DVector::zeros(2);
        actuation
            .write_generalized_velocity(&actuator_velocity, &mut generalized_velocity)
            .unwrap();
        assert_eq!(generalized_velocity.as_slice(), &[3.0, 2.0]);
        let mut round_trip = DVector::zeros(2);
        actuation
            .write_actuator_velocity(&generalized_velocity, &mut round_trip)
            .unwrap();
        assert_eq!(round_trip, actuator_velocity);

        let generalized_effort = DVector::from_vec(vec![8.0, -3.0]);
        let mut actuator_effort = DVector::zeros(2);
        actuation
            .write_actuator_effort(&generalized_effort, &mut actuator_effort)
            .unwrap();
        let generalized_power = generalized_effort.dot(&generalized_velocity);
        let actuator_power = actuator_effort.dot(&actuator_velocity);
        assert!((generalized_power - actuator_power).abs() < 1e-12);
    }
}
