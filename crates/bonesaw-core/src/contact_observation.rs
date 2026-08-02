//! Causal, allocation-free contact observation admission.
//!
//! Contact identity is caller-authored evidence. The filter never reads a
//! clock, estimates contact from kinematics, or advances without an explicit
//! call. Debounced mode state and hard-row eligibility remain separate: an
//! exact absent observation removes a hard contact immediately, while a new
//! contact must satisfy the configured activation evidence count.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ContactObservationStatus {
    Accepted = 0,
    Missing = 1,
    RejectedFuture = 2,
    RejectedStale = 3,
    RejectedUncertain = 4,
    RejectedSource = 5,
    RejectedSequence = 6,
    RejectedTimestamp = 7,
    InvalidConfig = 8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ContactObservationProvenance {
    Exact = 0,
    Held = 1,
    Unavailable = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactObservationConfig {
    pub expected_source_identity: u32,
    pub activation_samples: u16,
    pub deactivation_samples: u16,
    pub maximum_age_ns: u64,
    pub maximum_synchronization_uncertainty_ns: u64,
}

impl Default for ContactObservationConfig {
    fn default() -> Self {
        Self {
            expected_source_identity: 1,
            activation_samples: 3,
            deactivation_samples: 2,
            maximum_age_ns: 20_000_000,
            maximum_synchronization_uncertainty_ns: 2_000_000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactObservation<const N: usize> {
    pub mapped_time_ns: u64,
    pub source_sequence: u64,
    pub source_identity: u32,
    pub synchronization_uncertainty_ns: u64,
    pub active: [bool; N],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactObservationState<const N: usize> {
    pub initialized: bool,
    pub last_mapped_time_ns: u64,
    pub last_source_sequence: u64,
    pub debounced_contact: [bool; N],
    pub pending_value: [bool; N],
    pub pending_samples: [u16; N],
    pub transition_count: u32,
}

impl<const N: usize> Default for ContactObservationState<N> {
    fn default() -> Self {
        Self {
            initialized: false,
            last_mapped_time_ns: 0,
            last_source_sequence: 0,
            debounced_contact: [false; N],
            pending_value: [false; N],
            pending_samples: [0; N],
            transition_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactObservationOutput<const N: usize> {
    pub status: ContactObservationStatus,
    pub provenance: ContactObservationProvenance,
    pub accepted: bool,
    pub hard_constraint_eligible: bool,
    pub age_ns: u64,
    pub raw_contact: [bool; N],
    pub debounced_contact: [bool; N],
    pub hard_contact: [bool; N],
    pub pending_samples: [u16; N],
    pub transition_count: u32,
    pub flags: u32,
}

pub const CONTACT_OBSERVATION_MISSING: u32 = 1 << 0;
pub const CONTACT_OBSERVATION_FUTURE: u32 = 1 << 1;
pub const CONTACT_OBSERVATION_STALE: u32 = 1 << 2;
pub const CONTACT_OBSERVATION_UNCERTAIN: u32 = 1 << 3;
pub const CONTACT_OBSERVATION_SOURCE: u32 = 1 << 4;
pub const CONTACT_OBSERVATION_SEQUENCE: u32 = 1 << 5;
pub const CONTACT_OBSERVATION_TIMESTAMP: u32 = 1 << 6;
pub const CONTACT_OBSERVATION_DEBOUNCING: u32 = 1 << 7;
pub const CONTACT_OBSERVATION_TRANSITION: u32 = 1 << 8;
pub const CONTACT_OBSERVATION_INVALID_CONFIG: u32 = 1 << 9;

fn fallback_output<const N: usize>(
    tick_time_ns: u64,
    status: ContactObservationStatus,
    flags: u32,
    raw_contact: [bool; N],
    config: ContactObservationConfig,
    state: &ContactObservationState<N>,
) -> ContactObservationOutput<N> {
    let age_ns = if state.initialized {
        tick_time_ns.saturating_sub(state.last_mapped_time_ns)
    } else {
        u64::MAX
    };
    let provenance = if state.initialized && age_ns <= config.maximum_age_ns {
        ContactObservationProvenance::Held
    } else {
        ContactObservationProvenance::Unavailable
    };
    ContactObservationOutput {
        status,
        provenance,
        accepted: false,
        hard_constraint_eligible: false,
        age_ns,
        raw_contact,
        debounced_contact: state.debounced_contact,
        hard_contact: [false; N],
        pending_samples: state.pending_samples,
        transition_count: state.transition_count,
        flags,
    }
}

/// Admit one optional contact observation at a caller-supplied control time.
/// Invalid observations are atomic. Missing or held evidence never emits hard
/// contact rows. Exact absence removes a row immediately even if the stable
/// mode is still inside its deactivation debounce interval.
pub fn step_contact_observation<const N: usize>(
    tick_time_ns: u64,
    observation: Option<ContactObservation<N>>,
    config: ContactObservationConfig,
    state: &mut ContactObservationState<N>,
) -> ContactObservationOutput<N> {
    if N == 0
        || config.activation_samples == 0
        || config.deactivation_samples == 0
        || config.maximum_age_ns == 0
    {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::InvalidConfig,
            CONTACT_OBSERVATION_INVALID_CONFIG,
            [false; N],
            config,
            state,
        );
    }
    let Some(observation) = observation else {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::Missing,
            CONTACT_OBSERVATION_MISSING,
            [false; N],
            config,
            state,
        );
    };
    if observation.source_identity != config.expected_source_identity {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::RejectedSource,
            CONTACT_OBSERVATION_SOURCE,
            observation.active,
            config,
            state,
        );
    }
    if observation.mapped_time_ns > tick_time_ns {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::RejectedFuture,
            CONTACT_OBSERVATION_FUTURE,
            observation.active,
            config,
            state,
        );
    }
    let age_ns = tick_time_ns - observation.mapped_time_ns;
    if age_ns > config.maximum_age_ns {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::RejectedStale,
            CONTACT_OBSERVATION_STALE,
            observation.active,
            config,
            state,
        );
    }
    if observation.synchronization_uncertainty_ns > config.maximum_synchronization_uncertainty_ns {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::RejectedUncertain,
            CONTACT_OBSERVATION_UNCERTAIN,
            observation.active,
            config,
            state,
        );
    }
    if state.initialized && observation.source_sequence <= state.last_source_sequence {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::RejectedSequence,
            CONTACT_OBSERVATION_SEQUENCE,
            observation.active,
            config,
            state,
        );
    }
    if state.initialized && observation.mapped_time_ns <= state.last_mapped_time_ns {
        return fallback_output(
            tick_time_ns,
            ContactObservationStatus::RejectedTimestamp,
            CONTACT_OBSERVATION_TIMESTAMP,
            observation.active,
            config,
            state,
        );
    }

    let mut next = *state;
    let transition_count_before = next.transition_count;
    for contact in 0..N {
        let raw = observation.active[contact];
        if raw == next.debounced_contact[contact] {
            next.pending_value[contact] = raw;
            next.pending_samples[contact] = 0;
            continue;
        }
        if next.pending_samples[contact] == 0 || next.pending_value[contact] != raw {
            next.pending_value[contact] = raw;
            next.pending_samples[contact] = 1;
        } else {
            next.pending_samples[contact] = next.pending_samples[contact].saturating_add(1);
        }
        let threshold = if raw {
            config.activation_samples
        } else {
            config.deactivation_samples
        };
        if next.pending_samples[contact] >= threshold {
            next.debounced_contact[contact] = raw;
            next.pending_samples[contact] = 0;
            next.transition_count = next.transition_count.saturating_add(1);
        }
    }
    next.initialized = true;
    next.last_mapped_time_ns = observation.mapped_time_ns;
    next.last_source_sequence = observation.source_sequence;
    let mut flags = 0;
    if next.pending_samples.iter().any(|count| *count > 0) {
        flags |= CONTACT_OBSERVATION_DEBOUNCING;
    }
    if next.transition_count != transition_count_before {
        flags |= CONTACT_OBSERVATION_TRANSITION;
    }
    let mut hard_contact = next.debounced_contact;
    for contact in 0..N {
        hard_contact[contact] &= observation.active[contact];
    }
    *state = next;
    ContactObservationOutput {
        status: ContactObservationStatus::Accepted,
        provenance: ContactObservationProvenance::Exact,
        accepted: true,
        hard_constraint_eligible: true,
        age_ns,
        raw_contact: observation.active,
        debounced_contact: next.debounced_contact,
        hard_contact,
        pending_samples: next.pending_samples,
        transition_count: next.transition_count,
        flags,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(sequence: u64, time_ns: u64, active: [bool; 2]) -> ContactObservation<2> {
        ContactObservation {
            mapped_time_ns: time_ns,
            source_sequence: sequence,
            source_identity: 1,
            synchronization_uncertainty_ns: 100,
            active,
        }
    }

    #[test]
    fn activation_release_and_hard_eligibility_are_distinct() {
        let config = ContactObservationConfig::default();
        let mut state = ContactObservationState::<2>::default();
        for sequence in 1..=2 {
            let output = step_contact_observation(
                sequence * 5_000_000,
                Some(sample(sequence, sequence * 5_000_000, [true, true])),
                config,
                &mut state,
            );
            assert_eq!(output.debounced_contact, [false, false]);
            assert_eq!(output.hard_contact, [false, false]);
        }
        let active = step_contact_observation(
            15_000_000,
            Some(sample(3, 15_000_000, [true, true])),
            config,
            &mut state,
        );
        assert_eq!(active.debounced_contact, [true, true]);
        assert_eq!(active.hard_contact, [true, true]);

        let first_loss = step_contact_observation(
            20_000_000,
            Some(sample(4, 20_000_000, [true, false])),
            config,
            &mut state,
        );
        assert_eq!(first_loss.debounced_contact, [true, true]);
        assert_eq!(first_loss.hard_contact, [true, false]);
        let released = step_contact_observation(
            25_000_000,
            Some(sample(5, 25_000_000, [true, false])),
            config,
            &mut state,
        );
        assert_eq!(released.debounced_contact, [true, false]);
        assert_eq!(released.hard_contact, [true, false]);
    }

    #[test]
    fn chatter_and_mirrored_single_contact_are_deterministic() {
        let config = ContactObservationConfig::default();
        let mut state = ContactObservationState::<2>::default();
        let rows = [
            [true, false],
            [false, true],
            [true, false],
            [false, true],
            [false, true],
            [false, true],
        ];
        let mut output = step_contact_observation(0, None, config, &mut state);
        for (index, active) in rows.into_iter().enumerate() {
            let sequence = index as u64 + 1;
            output = step_contact_observation(
                sequence * 5_000_000,
                Some(sample(sequence, sequence * 5_000_000, active)),
                config,
                &mut state,
            );
        }
        assert_eq!(output.debounced_contact, [false, true]);
        assert_eq!(output.transition_count, 1);
    }

    #[test]
    fn rejected_and_missing_evidence_is_atomic_and_never_hard() {
        let config = ContactObservationConfig::default();
        let mut state = ContactObservationState::<2>::default();
        for sequence in 1..=3 {
            step_contact_observation(
                sequence * 5_000_000,
                Some(sample(sequence, sequence * 5_000_000, [true, true])),
                config,
                &mut state,
            );
        }
        let before = state;
        let duplicate = step_contact_observation(
            20_000_000,
            Some(sample(3, 20_000_000, [false, false])),
            config,
            &mut state,
        );
        assert_eq!(duplicate.status, ContactObservationStatus::RejectedSequence);
        assert_eq!(duplicate.hard_contact, [false, false]);
        assert_eq!(state, before);
        let held = step_contact_observation(20_000_000, None, config, &mut state);
        assert_eq!(held.provenance, ContactObservationProvenance::Held);
        assert_eq!(held.hard_contact, [false, false]);
        let expired = step_contact_observation(40_000_001, None, config, &mut state);
        assert_eq!(
            expired.provenance,
            ContactObservationProvenance::Unavailable
        );
        assert_eq!(state, before);
    }

    #[test]
    fn future_stale_uncertain_source_and_timestamp_faults_are_typed() {
        let config = ContactObservationConfig::default();
        let mut state = ContactObservationState::<2>::default();
        let valid = sample(1, 5_000_000, [true, true]);
        step_contact_observation(5_000_000, Some(valid), config, &mut state);
        let before = state;
        let mut cases = [
            (sample(2, 11_000_000, [true, true]), 10_000_000),
            (sample(2, 5_000_001, [true, true]), 30_000_002),
            (sample(2, 10_000_000, [true, true]), 10_000_000),
            (sample(2, 10_000_000, [true, true]), 10_000_000),
            (sample(2, 5_000_000, [true, true]), 10_000_000),
        ];
        cases[2].0.synchronization_uncertainty_ns = 2_000_001;
        cases[3].0.source_identity = 2;
        let expected = [
            ContactObservationStatus::RejectedFuture,
            ContactObservationStatus::RejectedStale,
            ContactObservationStatus::RejectedUncertain,
            ContactObservationStatus::RejectedSource,
            ContactObservationStatus::RejectedTimestamp,
        ];
        for ((observation, tick), status) in cases.into_iter().zip(expected) {
            let output = step_contact_observation(tick, Some(observation), config, &mut state);
            assert_eq!(output.status, status);
            assert_eq!(output.hard_contact, [false, false]);
            assert_eq!(state, before);
        }
    }
}
