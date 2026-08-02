//! Fixed-size proposal schedule for budgeted viability polling.
//!
//! The WBC and forecast scorer remain separate authorities. This module owns
//! only the deterministic next proposal, so Python evaluation code does not
//! carry mutable optimizer phase or create proposal arrays on the hot path.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityPollConfig<const N: usize> {
    pub maximum_abs_request: [f64; N],
    pub local_step: [f64; N],
}

impl<const N: usize> Default for ViabilityPollConfig<N> {
    fn default() -> Self {
        Self {
            maximum_abs_request: [1.0; N],
            local_step: [0.1; N],
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ViabilityPollState {
    pub phase: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityPollOutput<const N: usize> {
    pub proposal: [f64; N],
    pub axis: usize,
    pub direction: i8,
    pub phase: u32,
    pub saturated: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViabilityPollError {
    InvalidConfig,
    InvalidRequest,
}

/// Produce one signed coordinate proposal and advance the fixed cyclic phase.
///
/// Each coordinate receives a negative and positive question before the next
/// coordinate. Invalid input is atomic: output is rejected and phase does not
/// advance. Clipping is explicit in the returned `saturated` witness.
pub fn next_viability_poll<const N: usize>(
    previous_request: [f64; N],
    config: ViabilityPollConfig<N>,
    state: &mut ViabilityPollState,
) -> Result<ViabilityPollOutput<N>, ViabilityPollError> {
    if N == 0
        || config
            .maximum_abs_request
            .iter()
            .chain(config.local_step.iter())
            .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err(ViabilityPollError::InvalidConfig);
    }
    if previous_request.iter().enumerate().any(|(axis, value)| {
        !value.is_finite() || value.abs() > config.maximum_abs_request[axis] + 1.0e-12
    }) {
        return Err(ViabilityPollError::InvalidRequest);
    }

    let phase_count = N.checked_mul(2).ok_or(ViabilityPollError::InvalidConfig)?;
    let phase = state.phase as usize % phase_count;
    let axis = phase / 2;
    let direction = if phase % 2 == 0 { -1 } else { 1 };
    let unconstrained = previous_request[axis] + direction as f64 * config.local_step[axis];
    let bounded = unconstrained.clamp(
        -config.maximum_abs_request[axis],
        config.maximum_abs_request[axis],
    );
    let mut proposal = previous_request;
    proposal[axis] = bounded;
    let output = ViabilityPollOutput {
        proposal,
        axis,
        direction,
        phase: state.phase,
        saturated: bounded.to_bits() != unconstrained.to_bits(),
    };
    state.phase = ((phase + 1) % phase_count) as u32;
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> ViabilityPollConfig<3> {
        ViabilityPollConfig {
            maximum_abs_request: [40.0, 40.0, 20.0],
            local_step: [40.0, 40.0, 20.0],
        }
    }

    #[test]
    fn signed_coordinate_cycle_is_fixed_and_mirrored() {
        let mut state = ViabilityPollState::default();
        let expected = [
            (0, -1, [-40.0, 0.0, 0.0]),
            (0, 1, [40.0, 0.0, 0.0]),
            (1, -1, [0.0, -40.0, 0.0]),
            (1, 1, [0.0, 40.0, 0.0]),
            (2, -1, [0.0, 0.0, -20.0]),
            (2, 1, [0.0, 0.0, 20.0]),
        ];
        for (axis, direction, proposal) in expected {
            let output = next_viability_poll([0.0; 3], config(), &mut state).unwrap();
            assert_eq!((output.axis, output.direction), (axis, direction));
            assert_eq!(output.proposal, proposal);
            assert!(!output.saturated);
        }
        assert_eq!(state.phase, 0);
    }

    #[test]
    fn clipping_is_visible_and_invalid_input_is_atomic() {
        let mut state = ViabilityPollState { phase: 1 };
        let clipped = next_viability_poll([40.0, 0.0, 0.0], config(), &mut state).unwrap();
        assert_eq!(clipped.proposal, [40.0, 0.0, 0.0]);
        assert!(clipped.saturated);
        assert_eq!(state.phase, 2);

        let before = state;
        assert_eq!(
            next_viability_poll([f64::NAN, 0.0, 0.0], config(), &mut state),
            Err(ViabilityPollError::InvalidRequest)
        );
        assert_eq!(state, before);
    }
}
