//! Allocation-free hybrid-support admission for reduced viability requests.
//!
//! The guard does not predict contact. It requires the current exact support
//! mode to have existed for a bounded dwell, forbids an authored roll request
//! that opens the missing wheel, and requires both exact candidate-WBC normal
//! loads to remain material before a double-support request can proceed.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ViabilityHybridGuardStatus {
    #[default]
    Inactive = 0,
    Admitted = 1,
    RejectedEvidence = 2,
    RejectedSupportAge = 3,
    RejectedSupportDirection = 4,
    RejectedLoad = 5,
    RejectedInput = 6,
    RejectedSequence = 7,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityHybridGuardConfig {
    pub minimum_support_age_ticks: u32,
    pub minimum_double_support_load_fraction: f64,
    pub minimum_opening_roll_capture_pressure: f64,
    pub maximum_abs_request: [f64; 3],
}

impl Default for ViabilityHybridGuardConfig {
    fn default() -> Self {
        Self {
            minimum_support_age_ticks: 3,
            minimum_double_support_load_fraction: 0.05,
            minimum_opening_roll_capture_pressure: 0.20,
            maximum_abs_request: [40.0, 40.0, 20.0],
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ViabilityHybridGuardState {
    pub tick_initialized: bool,
    pub last_tick_sequence: u64,
    pub last_support_mask: u8,
    pub support_age_ticks: u32,
    pub status: ViabilityHybridGuardStatus,
    pub transition_count: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityHybridGuardOutput {
    pub status: ViabilityHybridGuardStatus,
    pub executable: bool,
    pub shadow_admissible: bool,
    pub support_mask: u8,
    pub support_age_ticks: u32,
    pub minimum_load_fraction: f64,
    pub transition_count: u32,
    pub flags: u32,
}

pub const VIABILITY_HYBRID_GUARD_ADMITTED: u32 = 1 << 0;
pub const VIABILITY_HYBRID_GUARD_EVIDENCE_REJECTED: u32 = 1 << 1;
pub const VIABILITY_HYBRID_GUARD_SUPPORT_AGE_REJECTED: u32 = 1 << 2;
pub const VIABILITY_HYBRID_GUARD_DIRECTION_REJECTED: u32 = 1 << 3;
pub const VIABILITY_HYBRID_GUARD_LOAD_REJECTED: u32 = 1 << 4;
pub const VIABILITY_HYBRID_GUARD_INPUT_REJECTED: u32 = 1 << 5;
pub const VIABILITY_HYBRID_GUARD_SEQUENCE_REJECTED: u32 = 1 << 6;
pub const VIABILITY_HYBRID_GUARD_SUPPORT_CHANGED: u32 = 1 << 7;
pub const VIABILITY_HYBRID_GUARD_TRANSITION: u32 = 1 << 8;

fn valid_config(config: ViabilityHybridGuardConfig) -> bool {
    config.minimum_support_age_ticks > 0
        && config.minimum_double_support_load_fraction.is_finite()
        && (0.0..0.5).contains(&config.minimum_double_support_load_fraction)
        && config.minimum_opening_roll_capture_pressure.is_finite()
        && config.minimum_opening_roll_capture_pressure > 0.0
        && config
            .maximum_abs_request
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
}

fn update_status(state: &mut ViabilityHybridGuardState, status: ViabilityHybridGuardStatus) -> u32 {
    if state.status == status {
        0
    } else {
        state.status = status;
        state.transition_count = state.transition_count.saturating_add(1);
        VIABILITY_HYBRID_GUARD_TRANSITION
    }
}

fn output(
    state: &ViabilityHybridGuardState,
    executable: bool,
    minimum_load_fraction: f64,
    flags: u32,
) -> ViabilityHybridGuardOutput {
    ViabilityHybridGuardOutput {
        status: state.status,
        executable,
        shadow_admissible: executable
            || state.status == ViabilityHybridGuardStatus::RejectedSupportDirection,
        support_mask: state.last_support_mask,
        support_age_ticks: state.support_age_ticks,
        minimum_load_fraction,
        transition_count: state.transition_count,
        flags,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn step_viability_hybrid_guard(
    tick_sequence: u64,
    observation_exact: bool,
    candidate_available: bool,
    support_mask: u8,
    signed_roll_capture_pressure: f64,
    request: [f64; 3],
    candidate_normal_force: [f64; 2],
    config: ViabilityHybridGuardConfig,
    state: &mut ViabilityHybridGuardState,
) -> ViabilityHybridGuardOutput {
    if !valid_config(config) {
        return ViabilityHybridGuardOutput {
            status: ViabilityHybridGuardStatus::RejectedInput,
            executable: false,
            shadow_admissible: false,
            support_mask: state.last_support_mask,
            support_age_ticks: state.support_age_ticks,
            minimum_load_fraction: 0.0,
            transition_count: state.transition_count,
            flags: VIABILITY_HYBRID_GUARD_INPUT_REJECTED,
        };
    }
    if state.tick_initialized && tick_sequence <= state.last_tick_sequence {
        return ViabilityHybridGuardOutput {
            status: ViabilityHybridGuardStatus::RejectedSequence,
            executable: false,
            shadow_admissible: false,
            support_mask: state.last_support_mask,
            support_age_ticks: state.support_age_ticks,
            minimum_load_fraction: 0.0,
            transition_count: state.transition_count,
            flags: VIABILITY_HYBRID_GUARD_SEQUENCE_REJECTED,
        };
    }
    let invalid = support_mask > 3
        || !signed_roll_capture_pressure.is_finite()
        || request.iter().enumerate().any(|(axis, value)| {
            !value.is_finite() || value.abs() > config.maximum_abs_request[axis] + 1.0e-12
        })
        || candidate_normal_force
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0);
    if invalid {
        let mut next = *state;
        next.tick_initialized = true;
        next.last_tick_sequence = tick_sequence;
        next.support_age_ticks = 0;
        let flags = update_status(&mut next, ViabilityHybridGuardStatus::RejectedInput)
            | VIABILITY_HYBRID_GUARD_INPUT_REJECTED;
        *state = next;
        return output(state, false, 0.0, flags);
    }

    let mut next = *state;
    next.tick_initialized = true;
    next.last_tick_sequence = tick_sequence;
    let support_changed = !state.tick_initialized || support_mask != state.last_support_mask;
    next.last_support_mask = support_mask;
    next.support_age_ticks = if observation_exact {
        if support_changed {
            1
        } else {
            state.support_age_ticks.saturating_add(1)
        }
    } else {
        0
    };
    if !observation_exact {
        let flags = update_status(&mut next, ViabilityHybridGuardStatus::RejectedEvidence)
            | VIABILITY_HYBRID_GUARD_EVIDENCE_REJECTED
            | if support_changed {
                VIABILITY_HYBRID_GUARD_SUPPORT_CHANGED
            } else {
                0
            };
        *state = next;
        return output(state, false, 0.0, flags);
    }
    if !candidate_available {
        let flags = update_status(&mut next, ViabilityHybridGuardStatus::Inactive)
            | if support_changed {
                VIABILITY_HYBRID_GUARD_SUPPORT_CHANGED
            } else {
                0
            };
        *state = next;
        return output(state, false, 0.0, flags);
    }
    if support_mask == 0 || next.support_age_ticks < config.minimum_support_age_ticks {
        let flags = update_status(&mut next, ViabilityHybridGuardStatus::RejectedSupportAge)
            | VIABILITY_HYBRID_GUARD_SUPPORT_AGE_REJECTED;
        *state = next;
        return output(state, false, 0.0, flags);
    }
    let direction_opens_support =
        (support_mask == 1 && request[0] < -1.0e-12) || (support_mask == 2 && request[0] > 1.0e-12);
    let opening_request_is_emergency_corrective = request[0] * signed_roll_capture_pressure
        < -1.0e-12
        && signed_roll_capture_pressure.abs() + 1.0e-12
            >= config.minimum_opening_roll_capture_pressure;
    if direction_opens_support && !opening_request_is_emergency_corrective {
        let flags = update_status(
            &mut next,
            ViabilityHybridGuardStatus::RejectedSupportDirection,
        ) | VIABILITY_HYBRID_GUARD_DIRECTION_REJECTED;
        *state = next;
        return output(state, false, 0.0, flags);
    }
    let total_load = candidate_normal_force[0] + candidate_normal_force[1];
    let minimum_load_fraction = if total_load > f64::EPSILON {
        candidate_normal_force[0].min(candidate_normal_force[1]) / total_load
    } else {
        0.0
    };
    if support_mask == 3
        && minimum_load_fraction + 1.0e-12 < config.minimum_double_support_load_fraction
    {
        let flags = update_status(&mut next, ViabilityHybridGuardStatus::RejectedLoad)
            | VIABILITY_HYBRID_GUARD_LOAD_REJECTED;
        *state = next;
        return output(state, false, minimum_load_fraction, flags);
    }
    let flags = update_status(&mut next, ViabilityHybridGuardStatus::Admitted)
        | VIABILITY_HYBRID_GUARD_ADMITTED;
    *state = next;
    output(state, true, minimum_load_fraction, flags)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step(
        tick: u64,
        mask: u8,
        candidate: bool,
        request: [f64; 3],
        force: [f64; 2],
        state: &mut ViabilityHybridGuardState,
    ) -> ViabilityHybridGuardOutput {
        step_viability_hybrid_guard(
            tick,
            true,
            candidate,
            mask,
            0.0,
            request,
            force,
            ViabilityHybridGuardConfig::default(),
            state,
        )
    }

    #[test]
    fn support_dwell_is_updated_even_without_a_candidate() {
        let mut state = ViabilityHybridGuardState::default();
        for tick in 1..3 {
            assert!(!step(tick, 3, false, [0.0; 3], [20.0; 2], &mut state).executable);
        }
        let admitted = step(3, 3, true, [0.0; 3], [20.0; 2], &mut state);
        assert!(admitted.executable);
        assert_eq!(admitted.support_age_ticks, 3);
    }

    #[test]
    fn single_support_opening_request_is_rejected_but_tangent_request_is_admitted() {
        let mut state = ViabilityHybridGuardState::default();
        for tick in 1..3 {
            step(tick, 1, false, [0.0; 3], [20.0, 0.0], &mut state);
        }
        let opening = step(3, 1, true, [-40.0, 0.0, 0.0], [20.0, 0.0], &mut state);
        assert_eq!(
            opening.status,
            ViabilityHybridGuardStatus::RejectedSupportDirection
        );
        assert!(opening.shadow_admissible);
        assert!(!opening.executable);
        let emergency = step_viability_hybrid_guard(
            4,
            true,
            true,
            1,
            0.30,
            [-40.0, 0.0, 0.0],
            [20.0, 0.0],
            ViabilityHybridGuardConfig::default(),
            &mut state,
        );
        assert!(emergency.executable);
        let tangent = step(5, 1, true, [0.0, 0.0, -20.0], [20.0, 0.0], &mut state);
        assert!(tangent.shadow_admissible);
        assert!(tangent.executable);
    }

    #[test]
    fn double_support_requires_material_load_on_both_wheels() {
        let mut state = ViabilityHybridGuardState::default();
        for tick in 1..3 {
            step(tick, 3, false, [0.0; 3], [20.0; 2], &mut state);
        }
        assert_eq!(
            step(3, 3, true, [0.0; 3], [40.0, 0.0], &mut state).status,
            ViabilityHybridGuardStatus::RejectedLoad
        );
        assert!(step(4, 3, true, [0.0; 3], [19.0, 21.0], &mut state).executable);
    }

    #[test]
    fn evidence_loss_and_support_change_restart_dwell() {
        let mut state = ViabilityHybridGuardState::default();
        for tick in 1..=3 {
            step(tick, 3, false, [0.0; 3], [20.0; 2], &mut state);
        }
        let lost = step_viability_hybrid_guard(
            4,
            false,
            false,
            3,
            0.0,
            [0.0; 3],
            [0.0; 2],
            ViabilityHybridGuardConfig::default(),
            &mut state,
        );
        assert_eq!(lost.status, ViabilityHybridGuardStatus::RejectedEvidence);
        assert_eq!(
            step(5, 1, false, [0.0; 3], [20.0, 0.0], &mut state).support_age_ticks,
            1
        );
    }

    #[test]
    fn reordered_tick_is_atomic_and_bad_input_fails_closed() {
        let mut state = ViabilityHybridGuardState::default();
        step(1, 3, false, [0.0; 3], [20.0; 2], &mut state);
        let snapshot = state;
        let reordered = step(1, 3, true, [0.0; 3], [20.0; 2], &mut state);
        assert_eq!(
            reordered.status,
            ViabilityHybridGuardStatus::RejectedSequence
        );
        assert_eq!(state, snapshot);
        let rejected = step(2, 3, true, [f64::NAN, 0.0, 0.0], [20.0; 2], &mut state);
        assert_eq!(rejected.status, ViabilityHybridGuardStatus::RejectedInput);
        assert!(!rejected.executable);
    }
}
