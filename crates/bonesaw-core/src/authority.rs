//! Typed, threshold-free physical authority evidence.
//!
//! These helpers deliberately report raw margins and utilizations. Warning and
//! derating thresholds belong to a versioned robot profile or evaluator, not
//! to the rigid-body solver kernel.

use nalgebra::DVector;
use thiserror::Error;

use crate::{
    model::{CompiledModel, RobotState},
    solver::VelocityBounds,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointPositionHeadroom {
    pub coordinate: usize,
    /// Signed distance to the nearest finite position limit. Negative means
    /// the oracle state is already outside the authored interval.
    pub margin_rad: f64,
    /// `margin_rad / (upper - lower)`. It is 0.5 at interval center and zero
    /// at either limit, which keeps the normalization explicit and reversible.
    pub fraction_of_range: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorEffortUtilization {
    pub coordinate: usize,
    pub absolute_effort: f64,
    /// Direction-specific absolute bound used for this effort sample.
    pub effort_limit: f64,
    pub utilization: f64,
    /// Signed remaining effort. Negative means the supplied sample exceeds the
    /// declared bound.
    pub headroom: f64,
}

#[derive(Debug, Error, PartialEq)]
pub enum AuthorityEvidenceError {
    #[error("authority evidence dimension mismatch")]
    Dimension,
    #[error("authority evidence contains NaN or infinity")]
    NonFinite,
    #[error("joint position limits are invalid")]
    InvalidJointLimits,
    #[error("actuator effort bounds are invalid")]
    InvalidEffortBounds,
}

pub fn minimum_joint_position_headroom(
    model: &CompiledModel,
    state: &RobotState,
) -> Result<Option<JointPositionHeadroom>, AuthorityEvidenceError> {
    if state.q.len() != model.dof {
        return Err(AuthorityEvidenceError::Dimension);
    }
    if state.q.iter().any(|value| !value.is_finite()) {
        return Err(AuthorityEvidenceError::NonFinite);
    }
    let mut minimum = None;
    for joint in &model.joints {
        let Some(coordinate) = joint.coordinate else {
            continue;
        };
        let span = joint.limit.upper - joint.limit.lower;
        if !span.is_finite() {
            continue;
        }
        if span <= 0.0 {
            return Err(AuthorityEvidenceError::InvalidJointLimits);
        }
        let margin =
            (state.q[coordinate] - joint.limit.lower).min(joint.limit.upper - state.q[coordinate]);
        let candidate = JointPositionHeadroom {
            coordinate,
            margin_rad: margin,
            fraction_of_range: margin / span,
        };
        if minimum.is_none_or(|current: JointPositionHeadroom| {
            candidate.fraction_of_range < current.fraction_of_range
        }) {
            minimum = Some(candidate);
        }
    }
    Ok(minimum)
}

pub fn maximum_actuator_effort_utilization(
    actuator_effort: &DVector<f64>,
    bounds: &VelocityBounds,
) -> Result<Option<ActuatorEffortUtilization>, AuthorityEvidenceError> {
    if actuator_effort.len() != bounds.lower.len() || actuator_effort.len() != bounds.upper.len() {
        return Err(AuthorityEvidenceError::Dimension);
    }
    let mut maximum = None;
    for coordinate in 0..actuator_effort.len() {
        let effort = actuator_effort[coordinate];
        if !effort.is_finite() {
            return Err(AuthorityEvidenceError::NonFinite);
        }
        let directional_bound = if effort >= 0.0 {
            bounds.upper[coordinate]
        } else {
            -bounds.lower[coordinate]
        };
        if directional_bound.is_infinite() && directional_bound.is_sign_positive() {
            continue;
        }
        if !directional_bound.is_finite() || directional_bound < 0.0 {
            return Err(AuthorityEvidenceError::InvalidEffortBounds);
        }
        let absolute_effort = effort.abs();
        if directional_bound == 0.0 {
            if absolute_effort > 1e-12 {
                return Err(AuthorityEvidenceError::InvalidEffortBounds);
            }
            let candidate = ActuatorEffortUtilization {
                coordinate,
                absolute_effort,
                effort_limit: 0.0,
                utilization: 0.0,
                headroom: 0.0,
            };
            if maximum.is_none_or(|current: ActuatorEffortUtilization| {
                candidate.utilization > current.utilization
            }) {
                maximum = Some(candidate);
            }
            continue;
        }
        let candidate = ActuatorEffortUtilization {
            coordinate,
            absolute_effort,
            effort_limit: directional_bound,
            utilization: absolute_effort / directional_bound,
            headroom: directional_bound - absolute_effort,
        };
        if maximum.is_none_or(|current: ActuatorEffortUtilization| {
            candidate.utilization > current.utilization
        }) {
            maximum = Some(candidate);
        }
    }
    Ok(maximum)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{TimingSpec, program::MotionProgram};

    const MODEL: &str = r#"
    <robot name="authority">
      <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
      <link name="tip"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
      <joint name="hinge" type="revolute">
        <parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
        <limit lower="-2" upper="2" velocity="5" effort="10"/>
      </joint>
    </robot>
    "#;

    #[test]
    fn reports_signed_joint_headroom_and_effort_utilization() {
        let program = MotionProgram::compile_urdf(MODEL, TimingSpec::default(), 1).unwrap();
        let mut state = RobotState::zeros(&program.model);
        state.q[0] = 1.5;
        let joint = minimum_joint_position_headroom(&program.model, &state)
            .unwrap()
            .unwrap();
        assert_eq!(joint.coordinate, 0);
        assert_eq!(joint.margin_rad, 0.5);
        assert_eq!(joint.fraction_of_range, 0.125);

        let effort = DVector::from_element(1, -8.0);
        let bounds = VelocityBounds {
            lower: DVector::from_element(1, -10.0),
            upper: DVector::from_element(1, 6.0),
        };
        let actuator = maximum_actuator_effort_utilization(&effort, &bounds)
            .unwrap()
            .unwrap();
        assert_eq!(actuator.coordinate, 0);
        assert_eq!(actuator.effort_limit, 10.0);
        assert_eq!(actuator.utilization, 0.8);
        assert_eq!(actuator.headroom, 2.0);
    }

    #[test]
    fn skips_unbounded_actuators_and_rejects_dimension_mismatch() {
        let effort = DVector::from_element(1, 1.0);
        let unbounded = VelocityBounds {
            lower: DVector::from_element(1, f64::NEG_INFINITY),
            upper: DVector::from_element(1, f64::INFINITY),
        };
        assert_eq!(
            maximum_actuator_effort_utilization(&effort, &unbounded).unwrap(),
            None
        );
        assert_eq!(
            maximum_actuator_effort_utilization(
                &DVector::zeros(2),
                &VelocityBounds {
                    lower: DVector::zeros(1),
                    upper: DVector::zeros(1),
                }
            ),
            Err(AuthorityEvidenceError::Dimension)
        );
    }

    #[test]
    fn reports_zero_availability_without_dividing_by_zero() {
        let actuator = maximum_actuator_effort_utilization(
            &DVector::zeros(1),
            &VelocityBounds {
                lower: DVector::zeros(1),
                upper: DVector::zeros(1),
            },
        )
        .unwrap()
        .unwrap();
        assert_eq!(actuator.effort_limit, 0.0);
        assert_eq!(actuator.utilization, 0.0);
        assert_eq!(actuator.headroom, 0.0);
    }
}
