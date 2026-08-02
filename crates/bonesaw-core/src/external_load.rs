//! Typed provenance for loads crossing the online plant boundary.
//!
//! A declared applied wrench, a measured impact impulse, and an unobserved
//! model reserve are different evidence.  This module makes that distinction
//! explicit without owning transport, a clock, contact estimation, or plant
//! integration.  Only [`ExternalLoadClass::DeclaredContinuousWrench`] is an
//! executable command; the other classes may be reported as evidence but must
//! never be smuggled through the same actuator path.

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum ExternalLoadSource {
    #[default]
    Unavailable = 0,
    InteractiveOperator = 1,
    EvaluationHarness = 2,
    SupervisorySystem = 3,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum ExternalLoadClass {
    #[default]
    Unavailable = 0,
    DeclaredContinuousWrench = 1,
    MeasuredImpactImpulse = 2,
    UnobservedModelReserve = 3,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum ExternalLoadFrame {
    #[default]
    Unavailable = 0,
    World = 1,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ExternalLoadProvenance {
    pub source: ExternalLoadSource,
    pub load_class: ExternalLoadClass,
    pub force_frame: ExternalLoadFrame,
    pub application_point_frame: ExternalLoadFrame,
}

impl ExternalLoadProvenance {
    pub const fn interactive_operator_wrench() -> Self {
        Self {
            source: ExternalLoadSource::InteractiveOperator,
            load_class: ExternalLoadClass::DeclaredContinuousWrench,
            force_frame: ExternalLoadFrame::World,
            application_point_frame: ExternalLoadFrame::World,
        }
    }

    pub const fn evaluation_fixture_wrench() -> Self {
        Self {
            source: ExternalLoadSource::EvaluationHarness,
            load_class: ExternalLoadClass::DeclaredContinuousWrench,
            force_frame: ExternalLoadFrame::World,
            application_point_frame: ExternalLoadFrame::World,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DeclaredExternalWrench {
    pub request_sequence: u64,
    pub provenance: ExternalLoadProvenance,
    pub force: [f64; 3],
    pub application_point: [f64; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExternalLoadError {
    InvalidLimit,
    MissingSource,
    NonExecutableClass,
    UnsupportedFrame,
    NonFinite,
    ForceLimit,
}

/// Validate an externally authored wrench before it crosses into a plant.
///
/// This deliberately does not validate the body-relative application-point
/// radius because body pose is plant-owned.  Transport or plant code must
/// perform that second check against the current measured body pose.
pub fn validate_declared_external_wrench(
    command: DeclaredExternalWrench,
    maximum_force_n: f64,
) -> Result<(), ExternalLoadError> {
    if !maximum_force_n.is_finite() || maximum_force_n < 0.0 {
        return Err(ExternalLoadError::InvalidLimit);
    }
    if command.provenance.source == ExternalLoadSource::Unavailable {
        return Err(ExternalLoadError::MissingSource);
    }
    if command.provenance.load_class != ExternalLoadClass::DeclaredContinuousWrench {
        return Err(ExternalLoadError::NonExecutableClass);
    }
    if command.provenance.force_frame != ExternalLoadFrame::World
        || command.provenance.application_point_frame != ExternalLoadFrame::World
    {
        return Err(ExternalLoadError::UnsupportedFrame);
    }
    if command
        .force
        .iter()
        .chain(command.application_point.iter())
        .any(|value| !value.is_finite())
    {
        return Err(ExternalLoadError::NonFinite);
    }
    let force_norm_squared = command.force.iter().map(|value| value * value).sum::<f64>();
    if force_norm_squared > maximum_force_n * maximum_force_n + 1.0e-12 {
        return Err(ExternalLoadError::ForceLimit);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn command(provenance: ExternalLoadProvenance) -> DeclaredExternalWrench {
        DeclaredExternalWrench {
            request_sequence: 7,
            provenance,
            force: [3.0, 4.0, 0.0],
            application_point: [0.0, 0.0, 0.5],
        }
    }

    #[test]
    fn independently_sourced_declared_wrenches_are_executable() {
        assert_eq!(
            validate_declared_external_wrench(
                command(ExternalLoadProvenance::interactive_operator_wrench()),
                5.0,
            ),
            Ok(())
        );
        assert_eq!(
            validate_declared_external_wrench(
                command(ExternalLoadProvenance::evaluation_fixture_wrench()),
                5.0,
            ),
            Ok(())
        );
    }

    #[test]
    fn impact_and_model_reserve_cannot_execute_as_declared_wrenches() {
        for load_class in [
            ExternalLoadClass::MeasuredImpactImpulse,
            ExternalLoadClass::UnobservedModelReserve,
        ] {
            let mut provenance = ExternalLoadProvenance::interactive_operator_wrench();
            provenance.load_class = load_class;
            assert_eq!(
                validate_declared_external_wrench(command(provenance), 8.0),
                Err(ExternalLoadError::NonExecutableClass)
            );
        }
    }

    #[test]
    fn source_frame_finiteness_and_force_are_independent_gates() {
        let mut value = command(ExternalLoadProvenance::interactive_operator_wrench());
        value.provenance.source = ExternalLoadSource::Unavailable;
        assert_eq!(
            validate_declared_external_wrench(value, 8.0),
            Err(ExternalLoadError::MissingSource)
        );

        value = command(ExternalLoadProvenance::interactive_operator_wrench());
        value.provenance.force_frame = ExternalLoadFrame::Unavailable;
        assert_eq!(
            validate_declared_external_wrench(value, 8.0),
            Err(ExternalLoadError::UnsupportedFrame)
        );

        value = command(ExternalLoadProvenance::interactive_operator_wrench());
        value.force[0] = f64::NAN;
        assert_eq!(
            validate_declared_external_wrench(value, 8.0),
            Err(ExternalLoadError::NonFinite)
        );

        value = command(ExternalLoadProvenance::interactive_operator_wrench());
        value.force = [5.001, 0.0, 0.0];
        assert_eq!(
            validate_declared_external_wrench(value, 5.0),
            Err(ExternalLoadError::ForceLimit)
        );
    }
}
