//! Allocation-free shadow confirmation for reduced-model viability proposals.
//!
//! A forecast proposal is not executable merely because one local query says
//! it improves a reduced objective. This transition requires repeated,
//! directionally consistent proposals under the same exact support evidence
//! before it can cross into the ordinary request supervisor.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ViabilityConfirmationStatus {
    #[default]
    Inactive = 0,
    Shadowing = 1,
    Confirmed = 2,
    RevokedEvidence = 3,
    RevokedSupport = 4,
    Expired = 5,
    RejectedSequence = 6,
    RejectedInput = 7,
    InvalidConfig = 8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityConfirmationConfig<const N: usize> {
    pub minimum_consistent_updates: u32,
    pub maximum_update_gap_ticks: u32,
    pub minimum_score_improvement: f64,
    pub minimum_normalized_alignment: f64,
    pub maximum_abs_candidate: [f64; N],
}

impl<const N: usize> Default for ViabilityConfirmationConfig<N> {
    fn default() -> Self {
        Self {
            minimum_consistent_updates: 2,
            maximum_update_gap_ticks: 1,
            minimum_score_improvement: 0.0,
            minimum_normalized_alignment: 0.5,
            maximum_abs_candidate: [1.0; N],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityConfirmationState<const N: usize> {
    pub tick_initialized: bool,
    pub last_tick_sequence: u64,
    pub has_shadow: bool,
    pub confirmed: bool,
    pub shadow_support_mask: u32,
    pub last_update_tick_sequence: u64,
    pub consistent_update_count: u32,
    pub shadow_candidate: [f64; N],
    pub baseline_score: f64,
    pub candidate_score: f64,
    pub status: ViabilityConfirmationStatus,
    pub transition_count: u32,
}

impl<const N: usize> Default for ViabilityConfirmationState<N> {
    fn default() -> Self {
        Self {
            tick_initialized: false,
            last_tick_sequence: 0,
            has_shadow: false,
            confirmed: false,
            shadow_support_mask: 0,
            last_update_tick_sequence: 0,
            consistent_update_count: 0,
            shadow_candidate: [0.0; N],
            baseline_score: 0.0,
            candidate_score: 0.0,
            status: ViabilityConfirmationStatus::Inactive,
            transition_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityConfirmationOutput<const N: usize> {
    pub status: ViabilityConfirmationStatus,
    pub executable: bool,
    pub has_shadow: bool,
    pub support_mask: u32,
    pub consistent_update_count: u32,
    pub shadow_age_ticks: u64,
    pub normalized_alignment: f64,
    pub score_improvement: f64,
    pub baseline_score: f64,
    pub candidate_score: f64,
    pub shadow_candidate: [f64; N],
    pub confirmed_candidate: [f64; N],
    pub transition_count: u32,
    pub flags: u32,
}

pub const VIABILITY_CONFIRMATION_SHADOWING: u32 = 1 << 0;
pub const VIABILITY_CONFIRMATION_CONFIRMED: u32 = 1 << 1;
pub const VIABILITY_CONFIRMATION_EVIDENCE_REVOKED: u32 = 1 << 2;
pub const VIABILITY_CONFIRMATION_SUPPORT_CHANGED: u32 = 1 << 3;
pub const VIABILITY_CONFIRMATION_EXPIRED: u32 = 1 << 4;
pub const VIABILITY_CONFIRMATION_SEQUENCE_REJECTED: u32 = 1 << 5;
pub const VIABILITY_CONFIRMATION_INPUT_REJECTED: u32 = 1 << 6;
pub const VIABILITY_CONFIRMATION_DIRECTION_RESET: u32 = 1 << 7;
pub const VIABILITY_CONFIRMATION_TRANSITION: u32 = 1 << 8;

fn valid_config<const N: usize>(config: ViabilityConfirmationConfig<N>) -> bool {
    N > 0
        && config.minimum_consistent_updates > 0
        && config.minimum_score_improvement.is_finite()
        && config.minimum_score_improvement >= 0.0
        && config.minimum_normalized_alignment.is_finite()
        && (-1.0..=1.0).contains(&config.minimum_normalized_alignment)
        && config
            .maximum_abs_candidate
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
}

fn set_status<const N: usize>(
    state: &mut ViabilityConfirmationState<N>,
    status: ViabilityConfirmationStatus,
) -> u32 {
    if state.status == status {
        0
    } else {
        state.status = status;
        state.transition_count = state.transition_count.saturating_add(1);
        VIABILITY_CONFIRMATION_TRANSITION
    }
}

fn reset_shadow<const N: usize>(state: &mut ViabilityConfirmationState<N>) {
    state.has_shadow = false;
    state.confirmed = false;
    state.shadow_support_mask = 0;
    state.last_update_tick_sequence = 0;
    state.consistent_update_count = 0;
    state.shadow_candidate = [0.0; N];
    state.baseline_score = 0.0;
    state.candidate_score = 0.0;
}

fn alignment<const N: usize>(left: [f64; N], right: [f64; N], maximum: [f64; N]) -> f64 {
    let mut dot = 0.0;
    let mut left_norm = 0.0;
    let mut right_norm = 0.0;
    for axis in 0..N {
        let a = left[axis] / maximum[axis];
        let b = right[axis] / maximum[axis];
        dot += a * b;
        left_norm += a * a;
        right_norm += b * b;
    }
    if left_norm <= f64::EPSILON || right_norm <= f64::EPSILON {
        -1.0
    } else {
        (dot / (left_norm.sqrt() * right_norm.sqrt())).clamp(-1.0, 1.0)
    }
}

fn output<const N: usize>(
    state: &ViabilityConfirmationState<N>,
    status: ViabilityConfirmationStatus,
    normalized_alignment: f64,
    flags: u32,
) -> ViabilityConfirmationOutput<N> {
    let age = if state.has_shadow {
        state
            .last_tick_sequence
            .saturating_sub(state.last_update_tick_sequence)
    } else {
        0
    };
    ViabilityConfirmationOutput {
        status,
        executable: state.confirmed,
        has_shadow: state.has_shadow,
        support_mask: state.shadow_support_mask,
        consistent_update_count: state.consistent_update_count,
        shadow_age_ticks: age,
        normalized_alignment,
        score_improvement: state.baseline_score - state.candidate_score,
        baseline_score: state.baseline_score,
        candidate_score: state.candidate_score,
        shadow_candidate: state.shadow_candidate,
        confirmed_candidate: if state.confirmed {
            state.shadow_candidate
        } else {
            [0.0; N]
        },
        transition_count: state.transition_count,
        flags,
    }
}

/// Advance one explicit forecast-confirmation tick.
#[allow(clippy::too_many_arguments)]
pub fn step_viability_confirmation<const N: usize>(
    tick_sequence: u64,
    observation_exact: bool,
    planner_update: bool,
    support_mask: u32,
    candidate_available: bool,
    baseline_score: f64,
    candidate_score: f64,
    candidate: [f64; N],
    config: ViabilityConfirmationConfig<N>,
    state: &mut ViabilityConfirmationState<N>,
) -> ViabilityConfirmationOutput<N> {
    if !valid_config(config) {
        return output(
            state,
            ViabilityConfirmationStatus::InvalidConfig,
            0.0,
            VIABILITY_CONFIRMATION_INPUT_REJECTED,
        );
    }
    if state.tick_initialized && tick_sequence <= state.last_tick_sequence {
        return output(
            state,
            ViabilityConfirmationStatus::RejectedSequence,
            0.0,
            VIABILITY_CONFIRMATION_SEQUENCE_REJECTED,
        );
    }
    let invalid = (!planner_update && candidate_available)
        || (planner_update
            && candidate_available
            && (!baseline_score.is_finite()
                || !candidate_score.is_finite()
                || baseline_score - candidate_score < config.minimum_score_improvement
                || candidate.iter().enumerate().any(|(axis, value)| {
                    !value.is_finite() || value.abs() > config.maximum_abs_candidate[axis] + 1.0e-12
                })));
    if invalid {
        let mut next = *state;
        next.tick_initialized = true;
        next.last_tick_sequence = tick_sequence;
        reset_shadow(&mut next);
        let flags = set_status(&mut next, ViabilityConfirmationStatus::RejectedInput)
            | VIABILITY_CONFIRMATION_INPUT_REJECTED;
        *state = next;
        return output(state, state.status, 0.0, flags);
    }

    let mut next = *state;
    next.tick_initialized = true;
    next.last_tick_sequence = tick_sequence;
    if !observation_exact {
        reset_shadow(&mut next);
        let flags = set_status(&mut next, ViabilityConfirmationStatus::RevokedEvidence)
            | VIABILITY_CONFIRMATION_EVIDENCE_REVOKED;
        *state = next;
        return output(state, state.status, 0.0, flags);
    }
    if !planner_update {
        if next.has_shadow
            && tick_sequence.saturating_sub(next.last_update_tick_sequence)
                > config.maximum_update_gap_ticks as u64
        {
            reset_shadow(&mut next);
            let flags = set_status(&mut next, ViabilityConfirmationStatus::Expired)
                | VIABILITY_CONFIRMATION_EXPIRED;
            *state = next;
            return output(state, state.status, 0.0, flags);
        }
        let status = if next.confirmed {
            ViabilityConfirmationStatus::Confirmed
        } else if next.has_shadow {
            ViabilityConfirmationStatus::Shadowing
        } else {
            ViabilityConfirmationStatus::Inactive
        };
        let flags = set_status(&mut next, status)
            | if next.confirmed {
                VIABILITY_CONFIRMATION_CONFIRMED
            } else if next.has_shadow {
                VIABILITY_CONFIRMATION_SHADOWING
            } else {
                0
            };
        *state = next;
        return output(state, state.status, 0.0, flags);
    }
    if !candidate_available {
        reset_shadow(&mut next);
        let flags = set_status(&mut next, ViabilityConfirmationStatus::Inactive);
        *state = next;
        return output(state, state.status, 0.0, flags);
    }

    let had_shadow = next.has_shadow;
    let support_changed = had_shadow && support_mask != next.shadow_support_mask;
    let gap_ok = had_shadow
        && tick_sequence.saturating_sub(next.last_update_tick_sequence)
            <= config.maximum_update_gap_ticks as u64;
    let normalized_alignment = if had_shadow {
        alignment(
            next.shadow_candidate,
            candidate,
            config.maximum_abs_candidate,
        )
    } else {
        1.0
    };
    let direction_ok =
        had_shadow && normalized_alignment + 1.0e-12 >= config.minimum_normalized_alignment;
    let consistent = had_shadow && !support_changed && gap_ok && direction_ok;

    next.has_shadow = true;
    next.confirmed = consistent
        && next.consistent_update_count.saturating_add(1) >= config.minimum_consistent_updates;
    next.shadow_support_mask = support_mask;
    next.last_update_tick_sequence = tick_sequence;
    next.consistent_update_count = if consistent {
        next.consistent_update_count.saturating_add(1)
    } else {
        1
    };
    next.shadow_candidate = candidate;
    next.baseline_score = baseline_score;
    next.candidate_score = candidate_score;
    let status = if support_changed {
        ViabilityConfirmationStatus::RevokedSupport
    } else if next.confirmed {
        ViabilityConfirmationStatus::Confirmed
    } else {
        ViabilityConfirmationStatus::Shadowing
    };
    let mut flags = set_status(&mut next, status);
    if next.confirmed {
        flags |= VIABILITY_CONFIRMATION_CONFIRMED;
    } else {
        flags |= VIABILITY_CONFIRMATION_SHADOWING;
    }
    if support_changed {
        flags |= VIABILITY_CONFIRMATION_SUPPORT_CHANGED;
    }
    if had_shadow && !support_changed && gap_ok && !direction_ok {
        flags |= VIABILITY_CONFIRMATION_DIRECTION_RESET;
    }
    *state = next;
    output(state, state.status, normalized_alignment, flags)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> ViabilityConfirmationConfig<3> {
        ViabilityConfirmationConfig {
            maximum_abs_candidate: [40.0, 40.0, 20.0],
            ..ViabilityConfirmationConfig::default()
        }
    }

    fn step(
        tick: u64,
        support: u32,
        candidate: [f64; 3],
        state: &mut ViabilityConfirmationState<3>,
    ) -> ViabilityConfirmationOutput<3> {
        step_viability_confirmation(
            tick,
            true,
            true,
            support,
            true,
            1.0,
            0.8,
            candidate,
            config(),
            state,
        )
    }

    #[test]
    fn two_support_consistent_updates_confirm() {
        let mut state = ViabilityConfirmationState::default();
        let first = step(1, 3, [40.0, 0.0, 0.0], &mut state);
        assert_eq!(first.status, ViabilityConfirmationStatus::Shadowing);
        assert!(!first.executable);
        let second = step(2, 3, [40.0, 0.0, 10.0], &mut state);
        assert_eq!(second.status, ViabilityConfirmationStatus::Confirmed);
        assert!(second.executable);
    }

    #[test]
    fn support_change_revokes_and_restarts_confirmation() {
        let mut state = ViabilityConfirmationState::default();
        step(1, 3, [40.0, 0.0, 0.0], &mut state);
        assert!(step(2, 3, [40.0, 0.0, 0.0], &mut state).executable);
        let changed = step(3, 1, [40.0, 0.0, 0.0], &mut state);
        assert_eq!(changed.status, ViabilityConfirmationStatus::RevokedSupport);
        assert!(!changed.executable);
        assert!(step(4, 1, [40.0, 0.0, 0.0], &mut state).executable);
    }

    #[test]
    fn opposing_direction_resets_shadow_count() {
        let mut state = ViabilityConfirmationState::default();
        step(1, 3, [40.0, 0.0, 0.0], &mut state);
        let opposite = step(2, 3, [-40.0, 0.0, 0.0], &mut state);
        assert!(!opposite.executable);
        assert_eq!(opposite.consistent_update_count, 1);
        assert_ne!(opposite.flags & VIABILITY_CONFIRMATION_DIRECTION_RESET, 0);
    }

    #[test]
    fn failed_update_and_evidence_loss_revoke_immediately() {
        let mut state = ViabilityConfirmationState::default();
        step(1, 3, [40.0, 0.0, 0.0], &mut state);
        step(2, 3, [40.0, 0.0, 0.0], &mut state);
        let failed = step_viability_confirmation(
            3,
            true,
            true,
            3,
            false,
            0.0,
            0.0,
            [0.0; 3],
            config(),
            &mut state,
        );
        assert_eq!(failed.status, ViabilityConfirmationStatus::Inactive);
        assert!(!failed.executable);
        step(4, 3, [40.0, 0.0, 0.0], &mut state);
        let revoked = step_viability_confirmation(
            5,
            false,
            false,
            3,
            false,
            0.0,
            0.0,
            [0.0; 3],
            config(),
            &mut state,
        );
        assert_eq!(revoked.status, ViabilityConfirmationStatus::RevokedEvidence);
    }

    #[test]
    fn reordered_tick_is_atomic_and_bad_score_fails_closed() {
        let mut state = ViabilityConfirmationState::default();
        step(10, 3, [40.0, 0.0, 0.0], &mut state);
        let snapshot = state;
        let reordered = step(10, 3, [40.0, 0.0, 0.0], &mut state);
        assert_eq!(
            reordered.status,
            ViabilityConfirmationStatus::RejectedSequence
        );
        assert_eq!(state, snapshot);
        let rejected = step_viability_confirmation(
            11,
            true,
            true,
            3,
            true,
            1.0,
            f64::NAN,
            [40.0, 0.0, 0.0],
            config(),
            &mut state,
        );
        assert_eq!(rejected.status, ViabilityConfirmationStatus::RejectedInput);
        assert!(!rejected.has_shadow);
    }
}
