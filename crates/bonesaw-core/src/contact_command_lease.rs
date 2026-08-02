//! Bounded, allocation-free command retention across observed contact changes.
//!
//! This state machine does not infer contact, solve a controller, or authorize
//! a new support program. A fresh command may enter only when the caller has
//! already admitted it against exact contact evidence. The command may then be
//! retained for a bounded number of explicit control ticks only while the hard
//! contact mask differs from the mask under which it was authored.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ContactCommandLeaseStatus {
    #[default]
    Unavailable = 0,
    Fresh = 1,
    Leased = 2,
    Expired = 3,
    RevokedEvidence = 4,
    RevokedSupport = 5,
    RejectedSequence = 6,
    RejectedCommand = 7,
    InvalidConfig = 8,
    HeldInexactObservation = 9,
    FreshSupportFreeInexactObservation = 10,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ContactCommandProvenance {
    #[default]
    Unavailable = 0,
    Fresh = 1,
    Leased = 2,
    InexactObservationHold = 3,
    SupportFreeInexactObservation = 4,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ContactCommandLeaseConfig {
    /// Number of control ticks after the fresh tick during which the cached
    /// command may remain executable across a hard-contact change.
    pub maximum_hold_ticks: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactCommandLeaseState<const CONTACTS: usize, const COMMANDS: usize> {
    pub tick_initialized: bool,
    pub last_tick_sequence: u64,
    pub has_command: bool,
    pub fresh_tick_sequence: u64,
    pub authoring_contact: [bool; CONTACTS],
    pub command: [f64; COMMANDS],
    pub status: ContactCommandLeaseStatus,
    pub transition_count: u32,
}

impl<const CONTACTS: usize, const COMMANDS: usize> Default
    for ContactCommandLeaseState<CONTACTS, COMMANDS>
{
    fn default() -> Self {
        Self {
            tick_initialized: false,
            last_tick_sequence: 0,
            has_command: false,
            fresh_tick_sequence: 0,
            authoring_contact: [false; CONTACTS],
            command: [0.0; COMMANDS],
            status: ContactCommandLeaseStatus::Unavailable,
            transition_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactCommandLeaseOutput<const CONTACTS: usize, const COMMANDS: usize> {
    pub status: ContactCommandLeaseStatus,
    pub provenance: ContactCommandProvenance,
    pub executable: bool,
    pub command_age_ticks: u64,
    pub remaining_hold_ticks: u32,
    pub authoring_contact: [bool; CONTACTS],
    pub stable_contact: [bool; CONTACTS],
    pub hard_contact: [bool; CONTACTS],
    pub command: [f64; COMMANDS],
    pub transition_count: u32,
    pub flags: u32,
}

pub const CONTACT_COMMAND_FRESH: u32 = 1 << 0;
pub const CONTACT_COMMAND_LEASED: u32 = 1 << 1;
pub const CONTACT_COMMAND_STABLE_SUPPORT_CHANGED: u32 = 1 << 2;
pub const CONTACT_COMMAND_HARD_SUPPORT_CHANGED: u32 = 1 << 3;
pub const CONTACT_COMMAND_CONTACTLESS: u32 = 1 << 4;
pub const CONTACT_COMMAND_EXPIRED: u32 = 1 << 5;
pub const CONTACT_COMMAND_EVIDENCE_REVOKED: u32 = 1 << 6;
pub const CONTACT_COMMAND_SUPPORT_REVOKED: u32 = 1 << 7;
pub const CONTACT_COMMAND_SEQUENCE_REJECTED: u32 = 1 << 8;
pub const CONTACT_COMMAND_INVALID: u32 = 1 << 9;
pub const CONTACT_COMMAND_TRANSITION: u32 = 1 << 10;
pub const CONTACT_COMMAND_INEXACT_OBSERVATION_HELD: u32 = 1 << 11;
pub const CONTACT_COMMAND_SCALED: u32 = 1 << 12;
pub const CONTACT_COMMAND_INEXACT_OBSERVATION_FRESH: u32 = 1 << 13;

fn output<const CONTACTS: usize, const COMMANDS: usize>(
    status: ContactCommandLeaseStatus,
    provenance: ContactCommandProvenance,
    executable: bool,
    command_age_ticks: u64,
    remaining_hold_ticks: u32,
    stable_contact: [bool; CONTACTS],
    hard_contact: [bool; CONTACTS],
    state: &ContactCommandLeaseState<CONTACTS, COMMANDS>,
    mut flags: u32,
) -> ContactCommandLeaseOutput<CONTACTS, COMMANDS> {
    if stable_contact != state.authoring_contact {
        flags |= CONTACT_COMMAND_STABLE_SUPPORT_CHANGED;
    }
    if hard_contact != state.authoring_contact {
        flags |= CONTACT_COMMAND_HARD_SUPPORT_CHANGED;
    }
    if !hard_contact.iter().any(|active| *active) {
        flags |= CONTACT_COMMAND_CONTACTLESS;
    }
    ContactCommandLeaseOutput {
        status,
        provenance,
        executable,
        command_age_ticks,
        remaining_hold_ticks,
        authoring_contact: state.authoring_contact,
        stable_contact,
        hard_contact,
        command: if executable {
            state.command
        } else {
            [0.0; COMMANDS]
        },
        transition_count: state.transition_count,
        flags,
    }
}

fn set_status<const CONTACTS: usize, const COMMANDS: usize>(
    state: &mut ContactCommandLeaseState<CONTACTS, COMMANDS>,
    status: ContactCommandLeaseStatus,
) -> u32 {
    if state.status == status {
        0
    } else {
        state.status = status;
        state.transition_count = state.transition_count.saturating_add(1);
        CONTACT_COMMAND_TRANSITION
    }
}

/// Advance one explicit command-retention tick.
///
/// `fresh_command` must already have passed the caller's WBC and execution
/// admission. It refreshes the lease under the current hard-contact mask.
/// Without a fresh command, retention is possible only with exact observation
/// evidence, a changed hard-contact mask, and age inside `maximum_hold_ticks`.
/// A returned authoring mask without a newly admitted command revokes the old
/// command instead of resurrecting it.
pub fn step_contact_command_lease<const CONTACTS: usize, const COMMANDS: usize>(
    tick_sequence: u64,
    observation_exact: bool,
    stable_contact: [bool; CONTACTS],
    hard_contact: [bool; CONTACTS],
    fresh_command: Option<[f64; COMMANDS]>,
    config: ContactCommandLeaseConfig,
    state: &mut ContactCommandLeaseState<CONTACTS, COMMANDS>,
) -> ContactCommandLeaseOutput<CONTACTS, COMMANDS> {
    if CONTACTS == 0 || COMMANDS == 0 {
        return output(
            ContactCommandLeaseStatus::InvalidConfig,
            ContactCommandProvenance::Unavailable,
            false,
            0,
            0,
            stable_contact,
            hard_contact,
            state,
            CONTACT_COMMAND_INVALID,
        );
    }
    if state.tick_initialized && tick_sequence <= state.last_tick_sequence {
        return output(
            ContactCommandLeaseStatus::RejectedSequence,
            ContactCommandProvenance::Unavailable,
            false,
            tick_sequence.saturating_sub(state.fresh_tick_sequence),
            0,
            stable_contact,
            hard_contact,
            state,
            CONTACT_COMMAND_SEQUENCE_REJECTED,
        );
    }
    if fresh_command
        .as_ref()
        .is_some_and(|command| command.iter().any(|value| !value.is_finite()))
    {
        let mut next = *state;
        next.tick_initialized = true;
        next.last_tick_sequence = tick_sequence;
        next.has_command = false;
        let flags = set_status(&mut next, ContactCommandLeaseStatus::RejectedCommand)
            | CONTACT_COMMAND_INVALID;
        *state = next;
        return output(
            ContactCommandLeaseStatus::RejectedCommand,
            ContactCommandProvenance::Unavailable,
            false,
            0,
            0,
            stable_contact,
            hard_contact,
            state,
            flags,
        );
    }

    let mut next = *state;
    next.tick_initialized = true;
    next.last_tick_sequence = tick_sequence;
    if !observation_exact {
        next.has_command = false;
        let flags = set_status(&mut next, ContactCommandLeaseStatus::RevokedEvidence)
            | CONTACT_COMMAND_EVIDENCE_REVOKED;
        *state = next;
        return output(
            ContactCommandLeaseStatus::RevokedEvidence,
            ContactCommandProvenance::Unavailable,
            false,
            0,
            0,
            stable_contact,
            hard_contact,
            state,
            flags,
        );
    }
    if let Some(command) = fresh_command {
        next.has_command = true;
        next.fresh_tick_sequence = tick_sequence;
        next.authoring_contact = hard_contact;
        next.command = command;
        let flags = set_status(&mut next, ContactCommandLeaseStatus::Fresh) | CONTACT_COMMAND_FRESH;
        *state = next;
        return output(
            ContactCommandLeaseStatus::Fresh,
            ContactCommandProvenance::Fresh,
            true,
            0,
            config.maximum_hold_ticks,
            stable_contact,
            hard_contact,
            state,
            flags,
        );
    }
    if !next.has_command {
        let flags = set_status(&mut next, ContactCommandLeaseStatus::Unavailable);
        *state = next;
        return output(
            ContactCommandLeaseStatus::Unavailable,
            ContactCommandProvenance::Unavailable,
            false,
            0,
            0,
            stable_contact,
            hard_contact,
            state,
            flags,
        );
    }

    let age = tick_sequence.saturating_sub(next.fresh_tick_sequence);
    if hard_contact == next.authoring_contact {
        next.has_command = false;
        let flags = set_status(&mut next, ContactCommandLeaseStatus::RevokedSupport)
            | CONTACT_COMMAND_SUPPORT_REVOKED;
        *state = next;
        return output(
            ContactCommandLeaseStatus::RevokedSupport,
            ContactCommandProvenance::Unavailable,
            false,
            age,
            0,
            stable_contact,
            hard_contact,
            state,
            flags,
        );
    }
    if age > u64::from(config.maximum_hold_ticks) {
        next.has_command = false;
        let flags =
            set_status(&mut next, ContactCommandLeaseStatus::Expired) | CONTACT_COMMAND_EXPIRED;
        *state = next;
        return output(
            ContactCommandLeaseStatus::Expired,
            ContactCommandProvenance::Unavailable,
            false,
            age,
            0,
            stable_contact,
            hard_contact,
            state,
            flags,
        );
    }

    let remaining = config.maximum_hold_ticks - age as u32;
    let flags = set_status(&mut next, ContactCommandLeaseStatus::Leased) | CONTACT_COMMAND_LEASED;
    *state = next;
    output(
        ContactCommandLeaseStatus::Leased,
        ContactCommandProvenance::Leased,
        true,
        age,
        remaining,
        stable_contact,
        hard_contact,
        state,
        flags,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    type State = ContactCommandLeaseState<2, 3>;

    fn step(
        tick: u64,
        exact: bool,
        stable: [bool; 2],
        hard: [bool; 2],
        fresh: Option<[f64; 3]>,
        maximum_hold_ticks: u32,
        state: &mut State,
    ) -> ContactCommandLeaseOutput<2, 3> {
        step_contact_command_lease(
            tick,
            exact,
            stable,
            hard,
            fresh,
            ContactCommandLeaseConfig { maximum_hold_ticks },
            state,
        )
    }

    #[test]
    fn exact_contact_change_holds_only_for_the_bounded_age() {
        let mut state = State::default();
        let fresh = step(
            10,
            true,
            [true; 2],
            [true; 2],
            Some([1.0, 2.0, 3.0]),
            2,
            &mut state,
        );
        assert_eq!(fresh.status, ContactCommandLeaseStatus::Fresh);
        assert_eq!(fresh.command, [1.0, 2.0, 3.0]);

        let pending = step(11, true, [true; 2], [true, false], None, 2, &mut state);
        assert_eq!(pending.status, ContactCommandLeaseStatus::Leased);
        assert_eq!(pending.remaining_hold_ticks, 1);
        assert_eq!(pending.command, [1.0, 2.0, 3.0]);
        assert_eq!(pending.flags & CONTACT_COMMAND_STABLE_SUPPORT_CHANGED, 0);

        let stable = step(12, true, [true, false], [true, false], None, 2, &mut state);
        assert_eq!(stable.status, ContactCommandLeaseStatus::Leased);
        assert_eq!(stable.remaining_hold_ticks, 0);
        assert_ne!(stable.flags & CONTACT_COMMAND_STABLE_SUPPORT_CHANGED, 0);

        let expired = step(13, true, [true, false], [true, false], None, 2, &mut state);
        assert_eq!(expired.status, ContactCommandLeaseStatus::Expired);
        assert!(!expired.executable);
        assert_eq!(expired.command, [0.0; 3]);
    }

    #[test]
    fn returned_support_requires_a_new_admitted_command() {
        let mut state = State::default();
        step(1, true, [true; 2], [true; 2], Some([1.0; 3]), 8, &mut state);
        assert!(step(2, true, [true, false], [true, false], None, 8, &mut state).executable);
        let returned = step(3, true, [true; 2], [true; 2], None, 8, &mut state);
        assert_eq!(returned.status, ContactCommandLeaseStatus::RevokedSupport);
        assert!(!returned.executable);
        let unavailable = step(4, true, [true, false], [true, false], None, 8, &mut state);
        assert_eq!(unavailable.status, ContactCommandLeaseStatus::Unavailable);
    }

    #[test]
    fn missing_evidence_revokes_without_resurrection() {
        let mut state = State::default();
        step(1, true, [true; 2], [true; 2], Some([1.0; 3]), 8, &mut state);
        let revoked = step(2, false, [true; 2], [true, false], None, 8, &mut state);
        assert_eq!(revoked.status, ContactCommandLeaseStatus::RevokedEvidence);
        assert!(!revoked.executable);
        let restored = step(3, true, [true, false], [true, false], None, 8, &mut state);
        assert_eq!(restored.status, ContactCommandLeaseStatus::Unavailable);
    }

    #[test]
    fn reordered_tick_is_atomic_and_invalid_command_is_non_executable() {
        let mut state = State::default();
        step(5, true, [true; 2], [true; 2], Some([1.0; 3]), 8, &mut state);
        let before = state;
        let duplicate = step(5, true, [true, false], [true, false], None, 8, &mut state);
        assert_eq!(
            duplicate.status,
            ContactCommandLeaseStatus::RejectedSequence
        );
        assert_eq!(state, before);

        let invalid = step(
            6,
            true,
            [true; 2],
            [true; 2],
            Some([f64::NAN; 3]),
            8,
            &mut state,
        );
        assert_eq!(invalid.status, ContactCommandLeaseStatus::RejectedCommand);
        assert!(!invalid.executable);
        assert!(!state.has_command);
    }

    #[test]
    fn zero_hold_configuration_never_replays() {
        let mut state = State::default();
        step(1, true, [true; 2], [true; 2], Some([1.0; 3]), 0, &mut state);
        let output = step(2, true, [true; 2], [true, false], None, 0, &mut state);
        assert_eq!(output.status, ContactCommandLeaseStatus::Expired);
        assert!(!output.executable);
    }
}
