//! Fail-closed evidence for reacquiring a previously lost contact.
//!
//! This observer is deliberately downstream of contact observation and
//! upstream of command authority.  It does not infer contact, solve a WBC,
//! or emit an effort.  A caller supplies an explicit target support mask,
//! the raw/stable/hard masks owned by [`crate::contact_observation`], and
//! measured normal loads.  Only an exact target mask with finite, positive
//! load on every target contact can accumulate reacquisition evidence.
//!
//! The first exact target sample after startup is a baseline, not a
//! reacquisition.  A target loss arms the observer; the target must then be
//! observed with valid load for `required_samples` consecutive, monotonic
//! ticks before `qualified` becomes true.  Qualification is a witness for a
//! future independently admitted command, never command authority itself.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ContactReacquisitionStatus {
    /// No target loss has been observed since startup or the last target
    /// change.  An exact target sample is only a baseline in this state.
    Baseline = 0,
    /// A valid exact observation showed that the target mask was absent or
    /// changed.  The observer is waiting for a valid target return.
    Armed = 1,
    /// Target mask and contact evidence are valid, but the consecutive-sample
    /// requirement has not yet been met.
    Candidate = 2,
    /// Target mask and measured loads have satisfied the full evidence gate.
    Qualified = 3,
    /// The caller supplied missing/inexact contact evidence or invalid loads.
    RejectedEvidence = 4,
    /// A valid observation was received, but the masks did not identify the
    /// requested target support.
    RejectedMask = 5,
    /// Loads were finite but did not satisfy the configured minimums.
    RejectedLoad = 6,
    /// The caller repeated or rewound the explicit tick sequence.
    RejectedSequence = 7,
    /// The observer configuration cannot safely run.
    InvalidConfig = 8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactReacquisitionConfig {
    /// Consecutive exact, load-qualified samples required after a loss.
    pub required_samples: u16,
    /// Minimum total measured normal load before per-contact fractions are
    /// considered meaningful.
    pub minimum_total_load_n: f64,
    /// Absolute minimum measured load for every active target contact.
    pub minimum_contact_load_n: f64,
    /// Minimum fraction of the total load carried by every active target
    /// contact.  This prevents a zero-force geometric touch from qualifying.
    pub minimum_load_fraction: f64,
}

impl Default for ContactReacquisitionConfig {
    fn default() -> Self {
        Self {
            required_samples: 3,
            minimum_total_load_n: 1.0,
            minimum_contact_load_n: 0.5,
            minimum_load_fraction: 0.05,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactReacquisitionState<const CONTACTS: usize> {
    pub tick_initialized: bool,
    pub last_tick_sequence: u64,
    pub target_initialized: bool,
    pub target_contact: [bool; CONTACTS],
    pub armed: bool,
    pub pending_samples: u16,
    pub qualified: bool,
    pub transition_count: u32,
}

impl<const CONTACTS: usize> Default for ContactReacquisitionState<CONTACTS> {
    fn default() -> Self {
        Self {
            tick_initialized: false,
            last_tick_sequence: 0,
            target_initialized: false,
            target_contact: [false; CONTACTS],
            armed: false,
            pending_samples: 0,
            qualified: false,
            transition_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactReacquisitionOutput<const CONTACTS: usize> {
    pub status: ContactReacquisitionStatus,
    pub observation_exact: bool,
    pub target_match: bool,
    pub masks_consistent: bool,
    pub load_valid: bool,
    pub armed: bool,
    pub qualified: bool,
    pub pending_samples: u16,
    pub transition_count: u32,
    pub total_load_n: f64,
    pub minimum_active_load_n: f64,
    pub weakest_load_fraction: f64,
    pub flags: u32,
}

pub const CONTACT_REACQUISITION_INVALID_CONFIG: u32 = 1 << 0;
pub const CONTACT_REACQUISITION_SEQUENCE_REJECTED: u32 = 1 << 1;
pub const CONTACT_REACQUISITION_EVIDENCE_REJECTED: u32 = 1 << 2;
pub const CONTACT_REACQUISITION_MASK_MISMATCH: u32 = 1 << 3;
pub const CONTACT_REACQUISITION_LOAD_REJECTED: u32 = 1 << 4;
pub const CONTACT_REACQUISITION_BASELINE: u32 = 1 << 5;
pub const CONTACT_REACQUISITION_ARMED: u32 = 1 << 6;
pub const CONTACT_REACQUISITION_CANDIDATE: u32 = 1 << 7;
pub const CONTACT_REACQUISITION_QUALIFIED: u32 = 1 << 8;
pub const CONTACT_REACQUISITION_TRANSITION: u32 = 1 << 9;
pub const CONTACT_REACQUISITION_TARGET_CHANGED: u32 = 1 << 10;

fn output<const CONTACTS: usize>(
    status: ContactReacquisitionStatus,
    observation_exact: bool,
    target_match: bool,
    masks_consistent: bool,
    load_valid: bool,
    total_load_n: f64,
    minimum_active_load_n: f64,
    weakest_load_fraction: f64,
    state: &ContactReacquisitionState<CONTACTS>,
    flags: u32,
) -> ContactReacquisitionOutput<CONTACTS> {
    ContactReacquisitionOutput {
        status,
        observation_exact,
        target_match,
        masks_consistent,
        load_valid,
        armed: state.armed,
        // Qualification is current-evidence authority, not a sticky state
        // bit.  A rejected or reordered sample must never expose a stale
        // `true` qualification to a caller.
        qualified: state.qualified && status == ContactReacquisitionStatus::Qualified,
        pending_samples: state.pending_samples,
        transition_count: state.transition_count,
        total_load_n,
        minimum_active_load_n,
        weakest_load_fraction,
        flags,
    }
}

fn valid_config(config: ContactReacquisitionConfig) -> bool {
    config.required_samples > 0
        && config.minimum_total_load_n.is_finite()
        && config.minimum_total_load_n > 0.0
        && config.minimum_contact_load_n.is_finite()
        && config.minimum_contact_load_n >= 0.0
        && config.minimum_load_fraction.is_finite()
        && config.minimum_load_fraction > 0.0
        && config.minimum_load_fraction <= 0.5
}

/// Advance one explicit reacquisition-observation tick.
pub fn step_contact_reacquisition<const CONTACTS: usize>(
    tick_sequence: u64,
    observation_exact: bool,
    target_contact: [bool; CONTACTS],
    raw_contact: [bool; CONTACTS],
    stable_contact: [bool; CONTACTS],
    hard_contact: [bool; CONTACTS],
    normal_load_n: [f64; CONTACTS],
    config: ContactReacquisitionConfig,
    state: &mut ContactReacquisitionState<CONTACTS>,
) -> ContactReacquisitionOutput<CONTACTS> {
    if CONTACTS == 0 || !valid_config(config) {
        return output(
            ContactReacquisitionStatus::InvalidConfig,
            observation_exact,
            false,
            false,
            false,
            0.0,
            0.0,
            0.0,
            state,
            CONTACT_REACQUISITION_INVALID_CONFIG,
        );
    }
    if state.tick_initialized && tick_sequence <= state.last_tick_sequence {
        return output(
            ContactReacquisitionStatus::RejectedSequence,
            observation_exact,
            false,
            false,
            false,
            0.0,
            0.0,
            0.0,
            state,
            CONTACT_REACQUISITION_SEQUENCE_REJECTED,
        );
    }

    // Invalid/missing evidence is atomic: a caller cannot make a target look
    // reacquired by sending NaNs, a stale mask, or an inexact observation.
    if !observation_exact
        || normal_load_n
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
    {
        return output(
            ContactReacquisitionStatus::RejectedEvidence,
            observation_exact,
            false,
            false,
            false,
            0.0,
            0.0,
            0.0,
            state,
            CONTACT_REACQUISITION_EVIDENCE_REJECTED,
        );
    }

    let masks_consistent = raw_contact == stable_contact && stable_contact == hard_contact;
    let target_match = masks_consistent && raw_contact == target_contact;
    let total_load_n = normal_load_n.iter().sum::<f64>();
    let mut minimum_active_load_n = f64::INFINITY;
    let mut weakest_load_fraction = f64::INFINITY;
    let mut load_valid = total_load_n >= config.minimum_total_load_n;
    for contact in 0..CONTACTS {
        if !target_contact[contact] {
            continue;
        }
        let load = normal_load_n[contact];
        minimum_active_load_n = minimum_active_load_n.min(load);
        let fraction = if total_load_n > 0.0 {
            load / total_load_n
        } else {
            0.0
        };
        weakest_load_fraction = weakest_load_fraction.min(fraction);
        load_valid &=
            load >= config.minimum_contact_load_n && fraction >= config.minimum_load_fraction;
    }
    if !minimum_active_load_n.is_finite() {
        minimum_active_load_n = 0.0;
        weakest_load_fraction = 0.0;
        load_valid = false;
    }

    let mut next = *state;
    next.tick_initialized = true;
    next.last_tick_sequence = tick_sequence;
    let mut flags = 0;
    if !next.target_initialized || next.target_contact != target_contact {
        next.target_initialized = true;
        next.target_contact = target_contact;
        next.armed = false;
        next.pending_samples = 0;
        next.qualified = false;
        next.transition_count = next.transition_count.saturating_add(1);
        flags |= CONTACT_REACQUISITION_TARGET_CHANGED | CONTACT_REACQUISITION_TRANSITION;
    }

    if !masks_consistent || !target_match {
        next.armed = true;
        next.pending_samples = 0;
        next.qualified = false;
        flags |= CONTACT_REACQUISITION_MASK_MISMATCH | CONTACT_REACQUISITION_ARMED;
        *state = next;
        return output(
            ContactReacquisitionStatus::Armed,
            observation_exact,
            target_match,
            masks_consistent,
            load_valid,
            total_load_n,
            minimum_active_load_n,
            weakest_load_fraction,
            state,
            flags,
        );
    }

    if !load_valid {
        next.pending_samples = 0;
        next.qualified = false;
        flags |= CONTACT_REACQUISITION_LOAD_REJECTED;
        *state = next;
        return output(
            ContactReacquisitionStatus::RejectedLoad,
            observation_exact,
            target_match,
            masks_consistent,
            false,
            total_load_n,
            minimum_active_load_n,
            weakest_load_fraction,
            state,
            flags,
        );
    }

    if !next.armed {
        next.pending_samples = 0;
        next.qualified = false;
        flags |= CONTACT_REACQUISITION_BASELINE;
        *state = next;
        return output(
            ContactReacquisitionStatus::Baseline,
            observation_exact,
            target_match,
            masks_consistent,
            true,
            total_load_n,
            minimum_active_load_n,
            weakest_load_fraction,
            state,
            flags,
        );
    }

    next.pending_samples = next.pending_samples.saturating_add(1);
    if next.pending_samples >= config.required_samples {
        next.qualified = true;
        flags |= CONTACT_REACQUISITION_QUALIFIED;
        if !state.qualified {
            next.transition_count = next.transition_count.saturating_add(1);
            flags |= CONTACT_REACQUISITION_TRANSITION;
        }
        *state = next;
        return output(
            ContactReacquisitionStatus::Qualified,
            observation_exact,
            target_match,
            masks_consistent,
            true,
            total_load_n,
            minimum_active_load_n,
            weakest_load_fraction,
            state,
            flags,
        );
    }

    flags |= CONTACT_REACQUISITION_CANDIDATE;
    *state = next;
    output(
        ContactReacquisitionStatus::Candidate,
        observation_exact,
        target_match,
        masks_consistent,
        true,
        total_load_n,
        minimum_active_load_n,
        weakest_load_fraction,
        state,
        flags,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    type State = ContactReacquisitionState<2>;

    fn config() -> ContactReacquisitionConfig {
        ContactReacquisitionConfig {
            required_samples: 3,
            minimum_total_load_n: 1.0,
            minimum_contact_load_n: 0.5,
            minimum_load_fraction: 0.05,
        }
    }

    fn step(
        tick: u64,
        target: [bool; 2],
        observed: [bool; 2],
        loads: [f64; 2],
        state: &mut State,
    ) -> ContactReacquisitionOutput<2> {
        step_contact_reacquisition(
            tick,
            true,
            target,
            observed,
            observed,
            observed,
            loads,
            config(),
            state,
        )
    }

    #[test]
    fn target_loss_then_three_loaded_samples_qualifies() {
        let target = [true, true];
        let mut state = State::default();
        let baseline = step(1, target, target, [25.0, 25.0], &mut state);
        assert_eq!(baseline.status, ContactReacquisitionStatus::Baseline);
        assert!(!baseline.qualified);

        let lost = step(2, target, [true, false], [25.0, 0.0], &mut state);
        assert_eq!(lost.status, ContactReacquisitionStatus::Armed);
        assert!(lost.armed);

        for (tick, expected) in [
            (3, ContactReacquisitionStatus::Candidate),
            (4, ContactReacquisitionStatus::Candidate),
            (5, ContactReacquisitionStatus::Qualified),
        ] {
            let output = step(tick, target, target, [20.0, 20.0], &mut state);
            assert_eq!(output.status, expected);
        }
        assert!(state.qualified);
        assert_eq!(state.pending_samples, 3);
    }

    #[test]
    fn geometric_touch_zero_load_and_inexact_evidence_never_qualify() {
        let target = [true, true];
        let mut state = State::default();
        step(1, target, target, [25.0, 25.0], &mut state);
        step(2, target, [false, false], [0.0, 0.0], &mut state);

        let touch = step(3, target, target, [20.0, 0.0], &mut state);
        assert_eq!(touch.status, ContactReacquisitionStatus::RejectedLoad);
        assert_eq!(state.pending_samples, 0);
        let inexact = step_contact_reacquisition(
            4,
            false,
            target,
            target,
            target,
            target,
            [20.0, 20.0],
            config(),
            &mut state,
        );
        assert_eq!(inexact.status, ContactReacquisitionStatus::RejectedEvidence);
        assert!(!inexact.qualified);
        assert_eq!(state.pending_samples, 0);
    }

    #[test]
    fn reordered_tick_is_atomic_and_target_change_restarts_baseline() {
        let target = [true, true];
        let mut state = State::default();
        step(1, target, target, [25.0, 25.0], &mut state);
        step(2, target, [false, false], [0.0, 0.0], &mut state);
        let candidate = step(3, target, target, [20.0, 20.0], &mut state);
        let before = state;
        let duplicate = step(3, target, target, [20.0, 20.0], &mut state);
        assert_eq!(
            duplicate.status,
            ContactReacquisitionStatus::RejectedSequence
        );
        assert_eq!(state, before);

        let changed = step(4, [true, false], [true, false], [25.0, 0.0], &mut state);
        assert_eq!(changed.status, ContactReacquisitionStatus::Baseline);
        assert!(!changed.qualified);
        assert!(candidate.pending_samples > 0);
    }

    #[test]
    fn rejected_evidence_does_not_publish_stale_qualification() {
        let target = [true, true];
        let mut state = State::default();
        step(1, target, target, [25.0, 25.0], &mut state);
        step(2, target, [false, false], [0.0, 0.0], &mut state);
        for tick in 3..=5 {
            step(tick, target, target, [20.0, 20.0], &mut state);
        }
        assert!(state.qualified);
        let rejected = step_contact_reacquisition(
            6,
            false,
            target,
            target,
            target,
            target,
            [20.0, 20.0],
            config(),
            &mut state,
        );
        assert_eq!(
            rejected.status,
            ContactReacquisitionStatus::RejectedEvidence
        );
        assert!(!rejected.qualified);
        assert!(state.qualified);
    }

    #[test]
    fn invalid_configuration_withholds_without_mutating_state() {
        let mut state = State::default();
        let before = state;
        let output = step_contact_reacquisition(
            1,
            true,
            [true, true],
            [true, true],
            [true, true],
            [true, true],
            [25.0, 25.0],
            ContactReacquisitionConfig {
                required_samples: 0,
                ..config()
            },
            &mut state,
        );
        assert_eq!(output.status, ContactReacquisitionStatus::InvalidConfig);
        assert!(!output.qualified);
        assert_eq!(state, before);
    }
}
