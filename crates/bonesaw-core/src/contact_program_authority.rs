//! Contact-program enable and reacquisition authority.
//!
//! Contact observation, program admission, and retained-command authority are
//! distinct facts. A newly solved command may become fresh only when raw,
//! stable, and hard contact masks agree. During activation or deactivation
//! debounce, authority may either retain a previously admitted command under
//! the bounded [`crate::contact_command_lease`] contract or, when explicitly
//! enabled after startup, accept a separately admitted current-hard-support
//! command. A separately configured inexact-observation hold may replay only
//! the last already-admitted effective command for a bounded number of ticks;
//! it is distinct from contact-transition retention. Otherwise authority is
//! withheld.

use crate::contact_command_lease::{
    CONTACT_COMMAND_CONTACTLESS, CONTACT_COMMAND_EXPIRED, CONTACT_COMMAND_HARD_SUPPORT_CHANGED,
    CONTACT_COMMAND_INEXACT_OBSERVATION_FRESH, CONTACT_COMMAND_INEXACT_OBSERVATION_HELD,
    CONTACT_COMMAND_INVALID, CONTACT_COMMAND_LEASED, CONTACT_COMMAND_SCALED,
    CONTACT_COMMAND_STABLE_SUPPORT_CHANGED, CONTACT_COMMAND_TRANSITION, ContactCommandLeaseConfig,
    ContactCommandLeaseOutput, ContactCommandLeaseState, ContactCommandLeaseStatus,
    ContactCommandProvenance, step_contact_command_lease,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ContactProgramSelection {
    #[default]
    Withheld = 0,
    FreshPrimary = 1,
    FreshCurrentSupport = 2,
    RetainedLease = 3,
    RetainedInexactObservation = 4,
    FreshSupportFreeInexactObservation = 5,
}

pub const INEXACT_OBSERVATION_HOLD_AUTHORITY_ONE_Q15: u16 = 32_768;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactProgramAuthorityConfig<const CONTACTS: usize> {
    pub primary_contact: [bool; CONTACTS],
    pub maximum_hold_ticks: u32,
    /// Number of consecutive non-exact observation ticks for which the last
    /// already-admitted effective command may remain executable. This is
    /// independent of contact-transition retention.
    pub maximum_inexact_observation_hold_ticks: u32,
    /// Q15 fraction of the last admitted effective effort emitted by the
    /// inexact-observation hold. `32768` is exactly one and `0` is zero effort.
    pub inexact_observation_hold_authority_q15: u16,
    /// Permit a separately current-hard-support-admitted command during a
    /// debounce transition after at least one prior fresh command exists.
    pub allow_transition_current_support: bool,
}

impl<const CONTACTS: usize> Default for ContactProgramAuthorityConfig<CONTACTS> {
    fn default() -> Self {
        Self {
            primary_contact: [true; CONTACTS],
            maximum_hold_ticks: 0,
            maximum_inexact_observation_hold_ticks: 0,
            inexact_observation_hold_authority_q15: INEXACT_OBSERVATION_HOLD_AUTHORITY_ONE_Q15,
            allow_transition_current_support: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactProgramAuthorityState<const CONTACTS: usize, const COMMANDS: usize> {
    pub lease: ContactCommandLeaseState<CONTACTS, COMMANDS>,
}

impl<const CONTACTS: usize, const COMMANDS: usize> Default
    for ContactProgramAuthorityState<CONTACTS, COMMANDS>
{
    fn default() -> Self {
        Self {
            lease: ContactCommandLeaseState::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactProgramAuthorityOutput<const CONTACTS: usize, const COMMANDS: usize> {
    pub selection: ContactProgramSelection,
    pub executable: bool,
    pub transition_pending: bool,
    pub activation_pending: bool,
    pub deactivation_pending: bool,
    pub masks_consistent: bool,
    pub lease: ContactCommandLeaseOutput<CONTACTS, COMMANDS>,
}

fn transition_flags<const CONTACTS: usize>(
    raw_contact: [bool; CONTACTS],
    stable_contact: [bool; CONTACTS],
) -> (bool, bool) {
    let mut activation = false;
    let mut deactivation = false;
    for index in 0..CONTACTS {
        activation |= raw_contact[index] && !stable_contact[index];
        deactivation |= !raw_contact[index] && stable_contact[index];
    }
    (activation, deactivation)
}

/// Select a fresh primary/current-support command or a bounded retained command.
///
/// `primary_command` and `current_support_command` are already-admitted command
/// candidates. This function never solves a controller or infers contact. A
/// fresh current-support candidate may enter while the observation filter is
/// transitioning only when the opt-in is set and prior authority exists.
pub fn step_contact_program_authority_with_inexact_command<
    const CONTACTS: usize,
    const COMMANDS: usize,
>(
    tick_sequence: u64,
    observation_exact: bool,
    raw_contact: [bool; CONTACTS],
    stable_contact: [bool; CONTACTS],
    hard_contact: [bool; CONTACTS],
    primary_command: Option<[f64; COMMANDS]>,
    current_support_command: Option<[f64; COMMANDS]>,
    support_free_inexact_observation_command: Option<[f64; COMMANDS]>,
    config: ContactProgramAuthorityConfig<CONTACTS>,
    state: &mut ContactProgramAuthorityState<CONTACTS, COMMANDS>,
) -> ContactProgramAuthorityOutput<CONTACTS, COMMANDS> {
    let (activation_pending, deactivation_pending) = transition_flags(raw_contact, stable_contact);
    let masks_consistent = (0..CONTACTS)
        .all(|index| hard_contact[index] == (raw_contact[index] && stable_contact[index]));
    let stable_authority = observation_exact
        && masks_consistent
        && raw_contact == stable_contact
        && hard_contact == stable_contact;
    let transition_current_authority = observation_exact
        && masks_consistent
        && (activation_pending || deactivation_pending)
        && config.allow_transition_current_support
        && state.lease.has_command
        && current_support_command.is_some();
    let (fresh_selection, fresh_command) = if stable_authority {
        if hard_contact == config.primary_contact {
            (ContactProgramSelection::FreshPrimary, primary_command)
        } else {
            (
                ContactProgramSelection::FreshCurrentSupport,
                current_support_command,
            )
        }
    } else if transition_current_authority {
        (
            ContactProgramSelection::FreshCurrentSupport,
            current_support_command,
        )
    } else {
        (ContactProgramSelection::Withheld, None)
    };
    let transition_pending = activation_pending || deactivation_pending;
    let lease_config = ContactCommandLeaseConfig {
        maximum_hold_ticks: config.maximum_hold_ticks,
    };
    let support_free_inexact_observation_command_invalid = !observation_exact
        && state.lease.has_command
        && support_free_inexact_observation_command
            .as_ref()
            .is_some_and(|command| command.iter().any(|value| !value.is_finite()))
        && (!state.lease.tick_initialized || tick_sequence > state.lease.last_tick_sequence);
    let support_free_inexact_observation = !observation_exact
        && state.lease.has_command
        && support_free_inexact_observation_command
            .as_ref()
            .is_some_and(|command| command.iter().all(|value| value.is_finite()))
        && (!state.lease.tick_initialized || tick_sequence > state.lease.last_tick_sequence);
    let inexact_observation_hold = !observation_exact
        && config.maximum_inexact_observation_hold_ticks > 0
        && config.inexact_observation_hold_authority_q15 > 0
        && config.inexact_observation_hold_authority_q15
            <= INEXACT_OBSERVATION_HOLD_AUTHORITY_ONE_Q15
        && state.lease.has_command
        && (!state.lease.tick_initialized || tick_sequence > state.lease.last_tick_sequence)
        && tick_sequence.saturating_sub(state.lease.fresh_tick_sequence)
            <= u64::from(config.maximum_inexact_observation_hold_ticks);
    let lease = if support_free_inexact_observation_command_invalid {
        let age = tick_sequence.saturating_sub(state.lease.fresh_tick_sequence);
        let mut flags = CONTACT_COMMAND_INVALID;
        if state.lease.status != ContactCommandLeaseStatus::RejectedCommand {
            state.lease.status = ContactCommandLeaseStatus::RejectedCommand;
            state.lease.transition_count = state.lease.transition_count.saturating_add(1);
            flags |= CONTACT_COMMAND_TRANSITION;
        }
        state.lease.tick_initialized = true;
        state.lease.last_tick_sequence = tick_sequence;
        ContactCommandLeaseOutput {
            status: ContactCommandLeaseStatus::RejectedCommand,
            provenance: ContactCommandProvenance::Unavailable,
            executable: false,
            command_age_ticks: age,
            remaining_hold_ticks: 0,
            authoring_contact: state.lease.authoring_contact,
            stable_contact,
            hard_contact,
            command: [0.0; COMMANDS],
            transition_count: state.lease.transition_count,
            flags,
        }
    } else if support_free_inexact_observation {
        let age = tick_sequence.saturating_sub(state.lease.fresh_tick_sequence);
        let mut flags = CONTACT_COMMAND_INEXACT_OBSERVATION_FRESH;
        if stable_contact != state.lease.authoring_contact {
            flags |= CONTACT_COMMAND_STABLE_SUPPORT_CHANGED;
        }
        if hard_contact != state.lease.authoring_contact {
            flags |= CONTACT_COMMAND_HARD_SUPPORT_CHANGED;
        }
        if !hard_contact.iter().any(|active| *active) {
            flags |= CONTACT_COMMAND_CONTACTLESS;
        }
        if state.lease.status != ContactCommandLeaseStatus::FreshSupportFreeInexactObservation {
            state.lease.status = ContactCommandLeaseStatus::FreshSupportFreeInexactObservation;
            state.lease.transition_count = state.lease.transition_count.saturating_add(1);
            flags |= CONTACT_COMMAND_TRANSITION;
        }
        state.lease.tick_initialized = true;
        state.lease.last_tick_sequence = tick_sequence;
        ContactCommandLeaseOutput {
            status: ContactCommandLeaseStatus::FreshSupportFreeInexactObservation,
            provenance: ContactCommandProvenance::SupportFreeInexactObservation,
            executable: true,
            command_age_ticks: age,
            remaining_hold_ticks: 0,
            authoring_contact: state.lease.authoring_contact,
            stable_contact,
            hard_contact,
            command: support_free_inexact_observation_command
                .expect("validated support-free inexact-observation command"),
            transition_count: state.lease.transition_count,
            flags,
        }
    } else if inexact_observation_hold {
        let age = tick_sequence.saturating_sub(state.lease.fresh_tick_sequence);
        let mut flags = CONTACT_COMMAND_INEXACT_OBSERVATION_HELD;
        if config.inexact_observation_hold_authority_q15
            < INEXACT_OBSERVATION_HOLD_AUTHORITY_ONE_Q15
        {
            flags |= CONTACT_COMMAND_SCALED;
        }
        if stable_contact != state.lease.authoring_contact {
            flags |= CONTACT_COMMAND_STABLE_SUPPORT_CHANGED;
        }
        if hard_contact != state.lease.authoring_contact {
            flags |= CONTACT_COMMAND_HARD_SUPPORT_CHANGED;
        }
        if !hard_contact.iter().any(|active| *active) {
            flags |= CONTACT_COMMAND_CONTACTLESS;
        }
        if state.lease.status != ContactCommandLeaseStatus::HeldInexactObservation {
            state.lease.status = ContactCommandLeaseStatus::HeldInexactObservation;
            state.lease.transition_count = state.lease.transition_count.saturating_add(1);
            flags |= CONTACT_COMMAND_TRANSITION;
        }
        state.lease.tick_initialized = true;
        state.lease.last_tick_sequence = tick_sequence;
        ContactCommandLeaseOutput {
            status: ContactCommandLeaseStatus::HeldInexactObservation,
            provenance: ContactCommandProvenance::InexactObservationHold,
            executable: true,
            command_age_ticks: age,
            remaining_hold_ticks: config.maximum_inexact_observation_hold_ticks - age as u32,
            authoring_contact: state.lease.authoring_contact,
            stable_contact,
            hard_contact,
            command: if config.inexact_observation_hold_authority_q15 == 0 {
                [0.0; COMMANDS]
            } else {
                std::array::from_fn(|index| {
                    state.lease.command[index]
                        * f64::from(config.inexact_observation_hold_authority_q15)
                        / f64::from(INEXACT_OBSERVATION_HOLD_AUTHORITY_ONE_Q15)
                })
            },
            transition_count: state.lease.transition_count,
            flags,
        }
    } else if observation_exact
        && masks_consistent
        && (transition_pending
            || (stable_authority && hard_contact != state.lease.authoring_contact))
        && !transition_current_authority
        && state.lease.has_command
        && (!state.lease.tick_initialized || tick_sequence > state.lease.last_tick_sequence)
    {
        let age = tick_sequence.saturating_sub(state.lease.fresh_tick_sequence);
        if age <= u64::from(config.maximum_hold_ticks) {
            let mut flags = CONTACT_COMMAND_LEASED;
            if stable_contact != state.lease.authoring_contact {
                flags |= CONTACT_COMMAND_STABLE_SUPPORT_CHANGED;
            }
            if hard_contact != state.lease.authoring_contact {
                flags |= CONTACT_COMMAND_HARD_SUPPORT_CHANGED;
            }
            if !hard_contact.iter().any(|active| *active) {
                flags |= CONTACT_COMMAND_CONTACTLESS;
            }
            if state.lease.status != ContactCommandLeaseStatus::Leased {
                state.lease.status = ContactCommandLeaseStatus::Leased;
                state.lease.transition_count = state.lease.transition_count.saturating_add(1);
                flags |= CONTACT_COMMAND_TRANSITION;
            }
            state.lease.tick_initialized = true;
            state.lease.last_tick_sequence = tick_sequence;
            ContactCommandLeaseOutput {
                status: ContactCommandLeaseStatus::Leased,
                provenance: ContactCommandProvenance::Leased,
                executable: true,
                command_age_ticks: age,
                remaining_hold_ticks: config.maximum_hold_ticks - age as u32,
                authoring_contact: state.lease.authoring_contact,
                stable_contact,
                hard_contact,
                command: state.lease.command,
                transition_count: state.lease.transition_count,
                flags,
            }
        } else if stable_authority && fresh_command.is_some() {
            step_contact_command_lease(
                tick_sequence,
                true,
                stable_contact,
                hard_contact,
                fresh_command,
                lease_config,
                &mut state.lease,
            )
        } else {
            let mut flags = CONTACT_COMMAND_EXPIRED;
            if state.lease.status != ContactCommandLeaseStatus::Expired {
                state.lease.status = ContactCommandLeaseStatus::Expired;
                state.lease.transition_count = state.lease.transition_count.saturating_add(1);
                flags |= CONTACT_COMMAND_TRANSITION;
            }
            state.lease.tick_initialized = true;
            state.lease.last_tick_sequence = tick_sequence;
            state.lease.has_command = false;
            ContactCommandLeaseOutput {
                status: ContactCommandLeaseStatus::Expired,
                provenance: ContactCommandProvenance::Unavailable,
                executable: false,
                command_age_ticks: age,
                remaining_hold_ticks: 0,
                authoring_contact: state.lease.authoring_contact,
                stable_contact,
                hard_contact,
                command: [0.0; COMMANDS],
                transition_count: state.lease.transition_count,
                flags,
            }
        }
    } else {
        step_contact_command_lease(
            tick_sequence,
            observation_exact && masks_consistent,
            stable_contact,
            hard_contact,
            fresh_command,
            lease_config,
            &mut state.lease,
        )
    };
    let selection = if lease.provenance == ContactCommandProvenance::Fresh && lease.executable {
        fresh_selection
    } else if lease.provenance == ContactCommandProvenance::SupportFreeInexactObservation
        && lease.executable
    {
        ContactProgramSelection::FreshSupportFreeInexactObservation
    } else if lease.provenance == ContactCommandProvenance::InexactObservationHold
        && lease.executable
    {
        ContactProgramSelection::RetainedInexactObservation
    } else if lease.provenance == ContactCommandProvenance::Leased && lease.executable {
        ContactProgramSelection::RetainedLease
    } else {
        ContactProgramSelection::Withheld
    };
    ContactProgramAuthorityOutput {
        selection,
        executable: lease.executable,
        transition_pending,
        activation_pending,
        deactivation_pending,
        masks_consistent,
        lease,
    }
}

/// Select authority without a separately admitted support-free command.
///
/// This compatibility entry point preserves the established primary/current/
/// retained semantics. Call [`step_contact_program_authority_with_inexact_command`]
/// only when a current-state command has independently passed a support-free
/// WBC admission path.
pub fn step_contact_program_authority<const CONTACTS: usize, const COMMANDS: usize>(
    tick_sequence: u64,
    observation_exact: bool,
    raw_contact: [bool; CONTACTS],
    stable_contact: [bool; CONTACTS],
    hard_contact: [bool; CONTACTS],
    primary_command: Option<[f64; COMMANDS]>,
    current_support_command: Option<[f64; COMMANDS]>,
    config: ContactProgramAuthorityConfig<CONTACTS>,
    state: &mut ContactProgramAuthorityState<CONTACTS, COMMANDS>,
) -> ContactProgramAuthorityOutput<CONTACTS, COMMANDS> {
    step_contact_program_authority_with_inexact_command(
        tick_sequence,
        observation_exact,
        raw_contact,
        stable_contact,
        hard_contact,
        primary_command,
        current_support_command,
        None,
        config,
        state,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    type State = ContactProgramAuthorityState<2, 2>;

    fn config() -> ContactProgramAuthorityConfig<2> {
        ContactProgramAuthorityConfig {
            primary_contact: [true, true],
            maximum_hold_ticks: 2,
            maximum_inexact_observation_hold_ticks: 0,
            inexact_observation_hold_authority_q15: INEXACT_OBSERVATION_HOLD_AUTHORITY_ONE_Q15,
            allow_transition_current_support: false,
        }
    }

    #[test]
    fn fresh_support_free_inexact_command_is_distinct_and_does_not_refresh_primary() {
        let mut state = State::default();
        let primary = step_contact_program_authority(
            10,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([1.0, 2.0]),
            None,
            config(),
            &mut state,
        );
        assert_eq!(primary.selection, ContactProgramSelection::FreshPrimary);

        let first = step_contact_program_authority_with_inexact_command(
            11,
            false,
            [true, true],
            [true, true],
            [false, false],
            None,
            None,
            Some([-0.5, 0.25]),
            config(),
            &mut state,
        );
        assert_eq!(
            first.selection,
            ContactProgramSelection::FreshSupportFreeInexactObservation
        );
        assert!(first.executable);
        assert_eq!(first.lease.command, [-0.5, 0.25]);
        assert_eq!(first.lease.command_age_ticks, 1);
        assert_eq!(state.lease.command, [1.0, 2.0]);
        assert_eq!(state.lease.fresh_tick_sequence, 10);

        let second = step_contact_program_authority_with_inexact_command(
            12,
            false,
            [true, true],
            [true, true],
            [false, false],
            None,
            None,
            Some([-0.25, 0.125]),
            config(),
            &mut state,
        );
        assert_eq!(
            second.selection,
            ContactProgramSelection::FreshSupportFreeInexactObservation
        );
        assert_eq!(second.lease.command, [-0.25, 0.125]);
        assert_eq!(second.lease.command_age_ticks, 2);
        assert_eq!(state.lease.command, [1.0, 2.0]);
        assert_eq!(state.lease.fresh_tick_sequence, 10);

        let recovered = step_contact_program_authority(
            13,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([3.0, 4.0]),
            None,
            config(),
            &mut state,
        );
        assert_eq!(recovered.selection, ContactProgramSelection::FreshPrimary);
        assert_eq!(recovered.lease.command, [3.0, 4.0]);
        assert_eq!(recovered.lease.command_age_ticks, 0);
    }

    #[test]
    fn invalid_support_free_inexact_command_withholds_without_falling_back() {
        let mut state = State::default();
        let hold_config = ContactProgramAuthorityConfig {
            maximum_inexact_observation_hold_ticks: 2,
            ..config()
        };
        let _ = step_contact_program_authority(
            20,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([1.0, 2.0]),
            None,
            hold_config,
            &mut state,
        );
        let invalid = step_contact_program_authority_with_inexact_command(
            21,
            false,
            [true, true],
            [true, true],
            [false, false],
            None,
            None,
            Some([f64::NAN, 0.0]),
            hold_config,
            &mut state,
        );
        assert_eq!(invalid.selection, ContactProgramSelection::Withheld);
        assert!(!invalid.executable);
        assert_eq!(
            invalid.lease.status,
            ContactCommandLeaseStatus::RejectedCommand
        );
        assert_eq!(invalid.lease.command, [0.0, 0.0]);
        assert_eq!(state.lease.command, [1.0, 2.0]);

        let recovered = step_contact_program_authority_with_inexact_command(
            22,
            false,
            [true, true],
            [true, true],
            [false, false],
            None,
            None,
            Some([-0.25, 0.125]),
            hold_config,
            &mut state,
        );
        assert_eq!(
            recovered.selection,
            ContactProgramSelection::FreshSupportFreeInexactObservation
        );
        assert_eq!(recovered.lease.command, [-0.25, 0.125]);
    }

    #[test]
    fn startup_activation_withholds_instead_of_executing_flight() {
        let mut state = State::default();
        let output = step_contact_program_authority(
            1,
            true,
            [true, true],
            [false, false],
            [false, false],
            Some([1.0, 2.0]),
            Some([3.0, 4.0]),
            config(),
            &mut state,
        );
        assert_eq!(output.selection, ContactProgramSelection::Withheld);
        assert!(output.activation_pending);
        assert!(!output.executable);
        assert_eq!(output.lease.command, [0.0, 0.0]);
    }

    #[test]
    fn transition_current_support_requires_prior_authority() {
        let mut state = State::default();
        let transition_config = ContactProgramAuthorityConfig {
            allow_transition_current_support: true,
            ..config()
        };
        let startup = step_contact_program_authority(
            1,
            true,
            [true, false],
            [true, true],
            [true, false],
            Some([1.0, 2.0]),
            Some([3.0, 4.0]),
            transition_config,
            &mut state,
        );
        assert_eq!(startup.selection, ContactProgramSelection::Withheld);
        assert!(!startup.executable);

        let primary = step_contact_program_authority(
            2,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([5.0, 6.0]),
            Some([7.0, 8.0]),
            transition_config,
            &mut state,
        );
        assert_eq!(primary.selection, ContactProgramSelection::FreshPrimary);

        let pending = step_contact_program_authority(
            3,
            true,
            [true, false],
            [true, true],
            [true, false],
            Some([9.0, 10.0]),
            Some([11.0, 12.0]),
            transition_config,
            &mut state,
        );
        assert_eq!(
            pending.selection,
            ContactProgramSelection::FreshCurrentSupport
        );
        assert!(pending.deactivation_pending);
        assert!(pending.executable);
        assert_eq!(pending.lease.authoring_contact, [true, false]);
        assert_eq!(pending.lease.command, [11.0, 12.0]);
    }

    #[test]
    fn stable_primary_then_loss_uses_lease_until_reduced_support_is_stable() {
        let mut state = State::default();
        let primary = step_contact_program_authority(
            1,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([1.0, 2.0]),
            Some([3.0, 4.0]),
            config(),
            &mut state,
        );
        assert_eq!(primary.selection, ContactProgramSelection::FreshPrimary);
        assert_eq!(primary.lease.command, [1.0, 2.0]);

        let pending = step_contact_program_authority(
            2,
            true,
            [true, false],
            [true, true],
            [true, false],
            Some([5.0, 6.0]),
            Some([7.0, 8.0]),
            config(),
            &mut state,
        );
        assert_eq!(pending.selection, ContactProgramSelection::RetainedLease);
        assert!(pending.deactivation_pending);
        assert_eq!(pending.lease.command, [1.0, 2.0]);

        let reduced = step_contact_program_authority(
            3,
            true,
            [true, false],
            [true, false],
            [true, false],
            Some([5.0, 6.0]),
            Some([7.0, 8.0]),
            config(),
            &mut state,
        );
        assert_eq!(reduced.selection, ContactProgramSelection::RetainedLease);
        assert_eq!(reduced.lease.command, [1.0, 2.0]);

        let reduced_fresh = step_contact_program_authority(
            4,
            true,
            [true, false],
            [true, false],
            [true, false],
            Some([5.0, 6.0]),
            Some([7.0, 8.0]),
            config(),
            &mut state,
        );
        assert_eq!(
            reduced_fresh.selection,
            ContactProgramSelection::FreshCurrentSupport
        );
        assert_eq!(reduced_fresh.lease.command, [7.0, 8.0]);

        let reacquiring_one = step_contact_program_authority(
            5,
            true,
            [true, true],
            [true, false],
            [true, false],
            Some([9.0, 10.0]),
            Some([11.0, 12.0]),
            config(),
            &mut state,
        );
        assert_eq!(
            reacquiring_one.selection,
            ContactProgramSelection::RetainedLease
        );
        assert_eq!(reacquiring_one.lease.command, [7.0, 8.0]);
        assert_eq!(reacquiring_one.lease.command_age_ticks, 1);

        let reacquiring_two = step_contact_program_authority(
            6,
            true,
            [true, true],
            [true, false],
            [true, false],
            Some([13.0, 14.0]),
            Some([15.0, 16.0]),
            config(),
            &mut state,
        );
        assert_eq!(
            reacquiring_two.selection,
            ContactProgramSelection::RetainedLease
        );
        assert_eq!(reacquiring_two.lease.command_age_ticks, 2);

        let reacquired = step_contact_program_authority(
            7,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([17.0, 18.0]),
            Some([19.0, 20.0]),
            config(),
            &mut state,
        );
        assert_eq!(reacquired.selection, ContactProgramSelection::FreshPrimary);
        assert_eq!(reacquired.lease.command, [17.0, 18.0]);
    }

    #[test]
    fn rejected_current_program_expires_old_support_command() {
        let mut state = State::default();
        let _ = step_contact_program_authority(
            10,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([1.0, 2.0]),
            None,
            config(),
            &mut state,
        );
        let leased = step_contact_program_authority(
            11,
            true,
            [true, false],
            [true, false],
            [true, false],
            None,
            None,
            config(),
            &mut state,
        );
        assert_eq!(leased.selection, ContactProgramSelection::RetainedLease);
        let expired = step_contact_program_authority(
            13,
            true,
            [true, false],
            [true, false],
            [true, false],
            None,
            None,
            config(),
            &mut state,
        );
        assert_eq!(expired.selection, ContactProgramSelection::Withheld);
        assert_eq!(
            expired.lease.status,
            crate::contact_command_lease::ContactCommandLeaseStatus::Expired
        );
    }

    #[test]
    fn evidence_loss_revokes_and_inconsistent_masks_cannot_execute() {
        let mut state = State::default();
        let _ = step_contact_program_authority(
            1,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([1.0, 2.0]),
            None,
            config(),
            &mut state,
        );
        let lost = step_contact_program_authority(
            2,
            false,
            [true, true],
            [true, true],
            [true, true],
            Some([3.0, 4.0]),
            None,
            config(),
            &mut state,
        );
        assert_eq!(lost.selection, ContactProgramSelection::Withheld);
        assert_eq!(
            lost.lease.status,
            crate::contact_command_lease::ContactCommandLeaseStatus::RevokedEvidence
        );

        let inconsistent = step_contact_program_authority(
            3,
            true,
            [false, false],
            [false, false],
            [true, false],
            None,
            Some([5.0, 6.0]),
            config(),
            &mut state,
        );
        assert!(!inconsistent.masks_consistent);
        assert!(!inconsistent.executable);
    }

    #[test]
    fn inexact_observation_hold_is_prior_bounded_and_distinct_from_transition_lease() {
        let mut state = State::default();
        let hold_config = ContactProgramAuthorityConfig {
            maximum_hold_ticks: 0,
            maximum_inexact_observation_hold_ticks: 2,
            ..config()
        };
        let startup = step_contact_program_authority(
            1,
            false,
            [true, true],
            [true, true],
            [true, true],
            Some([1.0, 2.0]),
            None,
            hold_config,
            &mut state,
        );
        assert_eq!(startup.selection, ContactProgramSelection::Withheld);
        assert!(!startup.executable);

        let fresh = step_contact_program_authority(
            2,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([3.0, 4.0]),
            None,
            hold_config,
            &mut state,
        );
        assert_eq!(fresh.selection, ContactProgramSelection::FreshPrimary);

        for (tick, remaining) in [(3, 1), (4, 0)] {
            let held = step_contact_program_authority(
                tick,
                false,
                [true, true],
                [true, true],
                [true, true],
                Some([5.0, 6.0]),
                None,
                hold_config,
                &mut state,
            );
            assert_eq!(
                held.selection,
                ContactProgramSelection::RetainedInexactObservation
            );
            assert_eq!(
                held.lease.status,
                ContactCommandLeaseStatus::HeldInexactObservation
            );
            assert_eq!(
                held.lease.provenance,
                ContactCommandProvenance::InexactObservationHold
            );
            assert_eq!(held.lease.command, [3.0, 4.0]);
            assert_eq!(held.lease.remaining_hold_ticks, remaining);
        }

        let expired = step_contact_program_authority(
            5,
            false,
            [true, true],
            [true, true],
            [true, true],
            Some([7.0, 8.0]),
            None,
            hold_config,
            &mut state,
        );
        assert_eq!(expired.selection, ContactProgramSelection::Withheld);
        assert_eq!(
            expired.lease.status,
            ContactCommandLeaseStatus::RevokedEvidence
        );
        assert_eq!(expired.lease.command, [0.0, 0.0]);

        let transition = step_contact_program_authority(
            6,
            true,
            [true, false],
            [true, true],
            [true, false],
            Some([9.0, 10.0]),
            None,
            hold_config,
            &mut state,
        );
        assert_eq!(transition.selection, ContactProgramSelection::Withheld);
        assert!(!transition.executable);
    }

    #[test]
    fn reordered_inexact_hold_tick_is_atomic() {
        let mut state = State::default();
        let hold_config = ContactProgramAuthorityConfig {
            maximum_inexact_observation_hold_ticks: 2,
            ..config()
        };
        let _ = step_contact_program_authority(
            10,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([1.0, 2.0]),
            None,
            hold_config,
            &mut state,
        );
        let first = step_contact_program_authority(
            11,
            false,
            [true, true],
            [true, true],
            [true, true],
            None,
            None,
            hold_config,
            &mut state,
        );
        assert_eq!(
            first.selection,
            ContactProgramSelection::RetainedInexactObservation
        );
        let retained_state = state;

        let reordered = step_contact_program_authority(
            11,
            false,
            [true, true],
            [true, true],
            [true, true],
            None,
            None,
            hold_config,
            &mut state,
        );
        assert_eq!(reordered.selection, ContactProgramSelection::Withheld);
        assert_eq!(
            reordered.lease.status,
            ContactCommandLeaseStatus::RejectedSequence
        );
        assert_eq!(state, retained_state);

        let second = step_contact_program_authority(
            12,
            false,
            [true, true],
            [true, true],
            [true, true],
            None,
            None,
            hold_config,
            &mut state,
        );
        assert_eq!(
            second.selection,
            ContactProgramSelection::RetainedInexactObservation
        );
        assert_eq!(second.lease.command_age_ticks, 2);
    }

    #[test]
    fn inexact_hold_authority_scales_cached_effort_without_compounding() {
        let mut state = State::default();
        let scaled_config = ContactProgramAuthorityConfig {
            maximum_inexact_observation_hold_ticks: 2,
            inexact_observation_hold_authority_q15: 8_192,
            ..config()
        };
        let _ = step_contact_program_authority(
            1,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([4.0, 8.0]),
            None,
            scaled_config,
            &mut state,
        );
        for tick in [2, 3] {
            let held = step_contact_program_authority(
                tick,
                false,
                [true, true],
                [true, true],
                [true, true],
                None,
                None,
                scaled_config,
                &mut state,
            );
            assert_eq!(held.lease.command, [1.0, 2.0]);
            assert_ne!(
                held.lease.flags & crate::contact_command_lease::CONTACT_COMMAND_SCALED,
                0
            );
        }
        assert_eq!(state.lease.command, [4.0, 8.0]);
    }

    #[test]
    fn zero_inexact_hold_authority_disables_retained_authority() {
        let mut state = State::default();
        let zero_config = ContactProgramAuthorityConfig {
            maximum_inexact_observation_hold_ticks: 1,
            inexact_observation_hold_authority_q15: 0,
            ..config()
        };
        let _ = step_contact_program_authority(
            1,
            true,
            [true, true],
            [true, true],
            [true, true],
            Some([-4.0, 8.0]),
            None,
            zero_config,
            &mut state,
        );
        let held = step_contact_program_authority(
            2,
            false,
            [true, true],
            [true, true],
            [true, true],
            None,
            None,
            zero_config,
            &mut state,
        );
        assert_eq!(held.selection, ContactProgramSelection::Withheld);
        assert!(!held.executable);
        assert_eq!(held.lease.command, [0.0, 0.0]);
        assert!(held.lease.command.iter().all(|value| value.to_bits() == 0));
    }
}
