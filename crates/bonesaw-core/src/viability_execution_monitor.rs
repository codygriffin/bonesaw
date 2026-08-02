//! Causal, allocation-free monitor for reduced execution-model residuals.
//!
//! The monitor never authorizes a command. It compares a prediction authored on
//! the previous tick with the current exact observation, evaluates that residual
//! against the envelope that existed *before* seeing it, then updates a bounded
//! rolling window. Support or evidence discontinuity resets the window so errors
//! from one hybrid mode cannot silently widen another mode's certificate.

pub const VIABILITY_EXECUTION_COMPONENTS: usize = 8;
pub const VIABILITY_EXECUTION_WINDOW: usize = 32;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ViabilityExecutionMonitorStatus {
    #[default]
    Warmup = 0,
    Covered = 1,
    Exceeded = 2,
    RejectedEvidence = 3,
    RejectedSupportChange = 4,
    RejectedInput = 5,
    RejectedSequence = 6,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityExecutionMonitorConfig {
    pub minimum_samples: u32,
    pub reserve_multiplier: f64,
    pub minimum_normalized_bound: [f64; VIABILITY_EXECUTION_COMPONENTS],
    pub maximum_normalized_bound: [f64; VIABILITY_EXECUTION_COMPONENTS],
}

impl Default for ViabilityExecutionMonitorConfig {
    fn default() -> Self {
        Self {
            minimum_samples: 8,
            reserve_multiplier: 2.0,
            minimum_normalized_bound: [1.0e-4; VIABILITY_EXECUTION_COMPONENTS],
            maximum_normalized_bound: [1.0; VIABILITY_EXECUTION_COMPONENTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityExecutionMonitorState {
    pub tick_initialized: bool,
    pub last_tick_sequence: u64,
    pub support_initialized: bool,
    pub support_mask: u8,
    pub sample_count: u32,
    pub write_index: usize,
    pub residuals: [[f64; VIABILITY_EXECUTION_COMPONENTS]; VIABILITY_EXECUTION_WINDOW],
    pub status: ViabilityExecutionMonitorStatus,
    pub transition_count: u32,
}

impl Default for ViabilityExecutionMonitorState {
    fn default() -> Self {
        Self {
            tick_initialized: false,
            last_tick_sequence: 0,
            support_initialized: false,
            support_mask: 0,
            sample_count: 0,
            write_index: 0,
            residuals: [[0.0; VIABILITY_EXECUTION_COMPONENTS]; VIABILITY_EXECUTION_WINDOW],
            status: ViabilityExecutionMonitorStatus::Warmup,
            transition_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityExecutionMonitorOutput {
    pub status: ViabilityExecutionMonitorStatus,
    pub certificate_valid: bool,
    pub support_mask: u8,
    pub sample_count: u32,
    pub normalized_error: [f64; VIABILITY_EXECUTION_COMPONENTS],
    pub normalized_bound: [f64; VIABILITY_EXECUTION_COMPONENTS],
    pub maximum_error_ratio: f64,
    pub transition_count: u32,
    pub flags: u32,
}

pub const VIABILITY_EXECUTION_COVERED: u32 = 1 << 0;
pub const VIABILITY_EXECUTION_EXCEEDED: u32 = 1 << 1;
pub const VIABILITY_EXECUTION_WARMUP: u32 = 1 << 2;
pub const VIABILITY_EXECUTION_EVIDENCE_REJECTED: u32 = 1 << 3;
pub const VIABILITY_EXECUTION_SUPPORT_CHANGED: u32 = 1 << 4;
pub const VIABILITY_EXECUTION_INPUT_REJECTED: u32 = 1 << 5;
pub const VIABILITY_EXECUTION_SEQUENCE_REJECTED: u32 = 1 << 6;
pub const VIABILITY_EXECUTION_PREDICTION_UNAVAILABLE: u32 = 1 << 7;
pub const VIABILITY_EXECUTION_TRANSITION: u32 = 1 << 8;

fn valid_config(config: ViabilityExecutionMonitorConfig) -> bool {
    config.minimum_samples > 0
        && config.minimum_samples as usize <= VIABILITY_EXECUTION_WINDOW
        && config.reserve_multiplier.is_finite()
        && config.reserve_multiplier >= 1.0
        && config
            .minimum_normalized_bound
            .iter()
            .zip(config.maximum_normalized_bound)
            .all(|(minimum, maximum)| {
                minimum.is_finite()
                    && *minimum >= 0.0
                    && maximum.is_finite()
                    && maximum > 0.0
                    && *minimum <= maximum
            })
}

fn update_status(
    state: &mut ViabilityExecutionMonitorState,
    status: ViabilityExecutionMonitorStatus,
) -> u32 {
    if state.status == status {
        0
    } else {
        state.status = status;
        state.transition_count = state.transition_count.saturating_add(1);
        VIABILITY_EXECUTION_TRANSITION
    }
}

fn reset_window(state: &mut ViabilityExecutionMonitorState) {
    state.sample_count = 0;
    state.write_index = 0;
    state.residuals = [[0.0; VIABILITY_EXECUTION_COMPONENTS]; VIABILITY_EXECUTION_WINDOW];
}

fn current_bound(
    state: &ViabilityExecutionMonitorState,
    config: ViabilityExecutionMonitorConfig,
) -> [f64; VIABILITY_EXECUTION_COMPONENTS] {
    let retained = (state.sample_count as usize).min(VIABILITY_EXECUTION_WINDOW);
    let mut bound = config.minimum_normalized_bound;
    for component in 0..VIABILITY_EXECUTION_COMPONENTS {
        let mut maximum = 0.0_f64;
        for sample in state.residuals.iter().take(retained) {
            maximum = maximum.max(sample[component]);
        }
        bound[component] = (config.reserve_multiplier * maximum)
            .max(config.minimum_normalized_bound[component])
            .min(config.maximum_normalized_bound[component]);
    }
    bound
}

fn output(
    state: &ViabilityExecutionMonitorState,
    certificate_valid: bool,
    normalized_error: [f64; VIABILITY_EXECUTION_COMPONENTS],
    normalized_bound: [f64; VIABILITY_EXECUTION_COMPONENTS],
    maximum_error_ratio: f64,
    flags: u32,
) -> ViabilityExecutionMonitorOutput {
    ViabilityExecutionMonitorOutput {
        status: state.status,
        certificate_valid,
        support_mask: state.support_mask,
        sample_count: state.sample_count,
        normalized_error,
        normalized_bound,
        maximum_error_ratio,
        transition_count: state.transition_count,
        flags,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn step_viability_execution_monitor(
    tick_sequence: u64,
    observation_exact: bool,
    prediction_available: bool,
    support_mask: u8,
    predicted_state: [f64; VIABILITY_EXECUTION_COMPONENTS],
    observed_state: [f64; VIABILITY_EXECUTION_COMPONENTS],
    normalization_scale: [f64; VIABILITY_EXECUTION_COMPONENTS],
    config: ViabilityExecutionMonitorConfig,
    state: &mut ViabilityExecutionMonitorState,
) -> ViabilityExecutionMonitorOutput {
    let existing_bound = current_bound(state, config);
    if !valid_config(config)
        || support_mask > 3
        || normalization_scale
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
        || (prediction_available
            && predicted_state
                .iter()
                .chain(observed_state.iter())
                .any(|value| !value.is_finite()))
    {
        return ViabilityExecutionMonitorOutput {
            status: ViabilityExecutionMonitorStatus::RejectedInput,
            certificate_valid: false,
            support_mask: state.support_mask,
            sample_count: state.sample_count,
            normalized_error: [0.0; VIABILITY_EXECUTION_COMPONENTS],
            normalized_bound: existing_bound,
            maximum_error_ratio: 0.0,
            transition_count: state.transition_count,
            flags: VIABILITY_EXECUTION_INPUT_REJECTED,
        };
    }
    if state.tick_initialized && tick_sequence <= state.last_tick_sequence {
        return ViabilityExecutionMonitorOutput {
            status: ViabilityExecutionMonitorStatus::RejectedSequence,
            certificate_valid: false,
            support_mask: state.support_mask,
            sample_count: state.sample_count,
            normalized_error: [0.0; VIABILITY_EXECUTION_COMPONENTS],
            normalized_bound: existing_bound,
            maximum_error_ratio: 0.0,
            transition_count: state.transition_count,
            flags: VIABILITY_EXECUTION_SEQUENCE_REJECTED,
        };
    }

    let mut next = *state;
    next.tick_initialized = true;
    next.last_tick_sequence = tick_sequence;
    if !observation_exact {
        reset_window(&mut next);
        next.support_initialized = false;
        let flags = update_status(&mut next, ViabilityExecutionMonitorStatus::RejectedEvidence)
            | VIABILITY_EXECUTION_EVIDENCE_REJECTED;
        *state = next;
        return output(
            state,
            false,
            [0.0; VIABILITY_EXECUTION_COMPONENTS],
            config.minimum_normalized_bound,
            0.0,
            flags,
        );
    }

    let support_changed = state.support_initialized && support_mask != state.support_mask;
    next.support_initialized = true;
    next.support_mask = support_mask;
    if support_changed {
        reset_window(&mut next);
        let flags = update_status(
            &mut next,
            ViabilityExecutionMonitorStatus::RejectedSupportChange,
        ) | VIABILITY_EXECUTION_SUPPORT_CHANGED;
        *state = next;
        return output(
            state,
            false,
            [0.0; VIABILITY_EXECUTION_COMPONENTS],
            config.minimum_normalized_bound,
            0.0,
            flags,
        );
    }
    if !prediction_available {
        let flags = update_status(&mut next, ViabilityExecutionMonitorStatus::Warmup)
            | VIABILITY_EXECUTION_WARMUP
            | VIABILITY_EXECUTION_PREDICTION_UNAVAILABLE;
        *state = next;
        return output(
            state,
            false,
            [0.0; VIABILITY_EXECUTION_COMPONENTS],
            current_bound(state, config),
            0.0,
            flags,
        );
    }

    let bound = current_bound(state, config);
    let mut error = [0.0; VIABILITY_EXECUTION_COMPONENTS];
    let mut maximum_ratio = 0.0_f64;
    let mut covered = true;
    for component in 0..VIABILITY_EXECUTION_COMPONENTS {
        error[component] = (observed_state[component] - predicted_state[component]).abs()
            / normalization_scale[component];
        maximum_ratio = maximum_ratio.max(error[component] / bound[component].max(f64::EPSILON));
        covered &= error[component] <= bound[component] + 1.0e-12;
    }
    let ready = state.sample_count >= config.minimum_samples;
    let status = if !ready {
        ViabilityExecutionMonitorStatus::Warmup
    } else if covered {
        ViabilityExecutionMonitorStatus::Covered
    } else {
        ViabilityExecutionMonitorStatus::Exceeded
    };
    let mut flags = update_status(&mut next, status);
    flags |= if !ready {
        VIABILITY_EXECUTION_WARMUP
    } else if covered {
        VIABILITY_EXECUTION_COVERED
    } else {
        VIABILITY_EXECUTION_EXCEEDED
    };
    next.residuals[next.write_index] = error;
    next.write_index = (next.write_index + 1) % VIABILITY_EXECUTION_WINDOW;
    next.sample_count = next.sample_count.saturating_add(1);
    *state = next;
    output(state, ready && covered, error, bound, maximum_ratio, flags)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step(
        tick: u64,
        support: u8,
        prediction_available: bool,
        predicted: [f64; VIABILITY_EXECUTION_COMPONENTS],
        observed: [f64; VIABILITY_EXECUTION_COMPONENTS],
        state: &mut ViabilityExecutionMonitorState,
    ) -> ViabilityExecutionMonitorOutput {
        let mut config = ViabilityExecutionMonitorConfig::default();
        config.minimum_samples = 2;
        config.minimum_normalized_bound = [0.01; VIABILITY_EXECUTION_COMPONENTS];
        step_viability_execution_monitor(
            tick,
            true,
            prediction_available,
            support,
            predicted,
            observed,
            [1.0; VIABILITY_EXECUTION_COMPONENTS],
            config,
            state,
        )
    }

    #[test]
    fn residual_is_checked_against_preupdate_bound() {
        let mut state = ViabilityExecutionMonitorState::default();
        step(1, 3, false, [0.0; 8], [0.0; 8], &mut state);
        step(2, 3, true, [0.0; 8], [0.005; 8], &mut state);
        step(3, 3, true, [0.0; 8], [0.005; 8], &mut state);
        let covered = step(4, 3, true, [0.0; 8], [0.005; 8], &mut state);
        assert_eq!(covered.status, ViabilityExecutionMonitorStatus::Covered);
        assert!(covered.certificate_valid);
        let exceeded = step(5, 3, true, [0.0; 8], [0.03; 8], &mut state);
        assert_eq!(exceeded.status, ViabilityExecutionMonitorStatus::Exceeded);
        assert!(!exceeded.certificate_valid);
        assert!(exceeded.maximum_error_ratio > 1.0);
        assert_eq!(exceeded.normalized_bound, [0.01; 8]);
    }

    #[test]
    fn support_and_evidence_discontinuity_reset_warmup() {
        let mut state = ViabilityExecutionMonitorState::default();
        step(1, 3, true, [0.0; 8], [0.005; 8], &mut state);
        step(2, 3, true, [0.0; 8], [0.005; 8], &mut state);
        let changed = step(3, 1, true, [0.0; 8], [0.005; 8], &mut state);
        assert_eq!(
            changed.status,
            ViabilityExecutionMonitorStatus::RejectedSupportChange
        );
        assert_eq!(changed.sample_count, 0);
        let lost = step_viability_execution_monitor(
            4,
            false,
            true,
            1,
            [0.0; 8],
            [0.0; 8],
            [1.0; 8],
            ViabilityExecutionMonitorConfig::default(),
            &mut state,
        );
        assert_eq!(
            lost.status,
            ViabilityExecutionMonitorStatus::RejectedEvidence
        );
        assert_eq!(lost.sample_count, 0);
    }

    #[test]
    fn reordered_and_invalid_inputs_are_atomic() {
        let mut state = ViabilityExecutionMonitorState::default();
        step(1, 3, false, [0.0; 8], [0.0; 8], &mut state);
        let snapshot = state;
        let reordered = step(1, 3, true, [0.0; 8], [0.0; 8], &mut state);
        assert_eq!(
            reordered.status,
            ViabilityExecutionMonitorStatus::RejectedSequence
        );
        assert_eq!(state, snapshot);
        let mut invalid_prediction = [0.0; 8];
        invalid_prediction[0] = f64::NAN;
        let invalid = step(2, 3, true, invalid_prediction, [0.0; 8], &mut state);
        assert_eq!(
            invalid.status,
            ViabilityExecutionMonitorStatus::RejectedInput
        );
        assert_eq!(state, snapshot);
    }
}
