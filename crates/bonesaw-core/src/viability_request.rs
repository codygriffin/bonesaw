//! Allocation-free supervision for a slower viability planner request.
//!
//! The planner, WBC, contact observer, and command executor remain separate
//! authorities. This state machine only turns explicit planner updates into a
//! bounded, slew-limited request between updates. Every executed request still
//! requires a fresh downstream WBC admission.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ViabilityRequestStatus {
    #[default]
    Inactive = 0,
    Fresh = 1,
    Held = 2,
    Releasing = 3,
    Expired = 4,
    RevokedEvidence = 5,
    RejectedSequence = 6,
    RejectedInput = 7,
    InvalidConfig = 8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ViabilityRequestProvenance {
    #[default]
    Unavailable = 0,
    Fresh = 1,
    Held = 2,
    Releasing = 3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityRequestConfig<const N: usize> {
    pub activation_pressure: f64,
    pub release_pressure: f64,
    pub maximum_candidate_age_ticks: u32,
    pub maximum_abs_request: [f64; N],
    pub maximum_slew_per_tick: [f64; N],
}

impl<const N: usize> Default for ViabilityRequestConfig<N> {
    fn default() -> Self {
        Self {
            activation_pressure: 0.10,
            release_pressure: 0.05,
            maximum_candidate_age_ticks: 4,
            maximum_abs_request: [1.0; N],
            maximum_slew_per_tick: [0.1; N],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityRequestState<const N: usize> {
    pub tick_initialized: bool,
    pub last_tick_sequence: u64,
    pub active: bool,
    pub has_candidate: bool,
    pub candidate_tick_sequence: u64,
    pub last_pressure: f64,
    pub target: [f64; N],
    pub request: [f64; N],
    pub status: ViabilityRequestStatus,
    pub transition_count: u32,
}

impl<const N: usize> Default for ViabilityRequestState<N> {
    fn default() -> Self {
        Self {
            tick_initialized: false,
            last_tick_sequence: 0,
            active: false,
            has_candidate: false,
            candidate_tick_sequence: 0,
            last_pressure: 0.0,
            target: [0.0; N],
            request: [0.0; N],
            status: ViabilityRequestStatus::Inactive,
            transition_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViabilityRequestOutput<const N: usize> {
    pub status: ViabilityRequestStatus,
    pub provenance: ViabilityRequestProvenance,
    pub active: bool,
    pub executable: bool,
    pub request_was_slew_limited: bool,
    pub candidate_age_ticks: u64,
    pub remaining_fresh_ticks: u32,
    pub last_pressure: f64,
    pub target: [f64; N],
    pub request: [f64; N],
    pub transition_count: u32,
    pub flags: u32,
}

pub const VIABILITY_REQUEST_ACTIVE: u32 = 1 << 0;
pub const VIABILITY_REQUEST_FRESH: u32 = 1 << 1;
pub const VIABILITY_REQUEST_HELD: u32 = 1 << 2;
pub const VIABILITY_REQUEST_RELEASING: u32 = 1 << 3;
pub const VIABILITY_REQUEST_EXPIRED: u32 = 1 << 4;
pub const VIABILITY_REQUEST_EVIDENCE_REVOKED: u32 = 1 << 5;
pub const VIABILITY_REQUEST_SEQUENCE_REJECTED: u32 = 1 << 6;
pub const VIABILITY_REQUEST_INPUT_REJECTED: u32 = 1 << 7;
pub const VIABILITY_REQUEST_SLEW_LIMITED: u32 = 1 << 8;
pub const VIABILITY_REQUEST_TRANSITION: u32 = 1 << 9;

fn valid_config<const N: usize>(config: ViabilityRequestConfig<N>) -> bool {
    N > 0
        && config.activation_pressure.is_finite()
        && config.release_pressure.is_finite()
        && config.release_pressure >= 0.0
        && config.release_pressure < config.activation_pressure
        && config
            .maximum_abs_request
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
        && config
            .maximum_slew_per_tick
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
}

fn set_status<const N: usize>(
    state: &mut ViabilityRequestState<N>,
    status: ViabilityRequestStatus,
) -> u32 {
    if state.status == status {
        0
    } else {
        state.status = status;
        state.transition_count = state.transition_count.saturating_add(1);
        VIABILITY_REQUEST_TRANSITION
    }
}

fn output<const N: usize>(
    state: &ViabilityRequestState<N>,
    status: ViabilityRequestStatus,
    provenance: ViabilityRequestProvenance,
    executable: bool,
    slew_limited: bool,
    age: u64,
    remaining: u32,
    mut flags: u32,
) -> ViabilityRequestOutput<N> {
    if state.active {
        flags |= VIABILITY_REQUEST_ACTIVE;
    }
    if slew_limited {
        flags |= VIABILITY_REQUEST_SLEW_LIMITED;
    }
    ViabilityRequestOutput {
        status,
        provenance,
        active: state.active,
        executable,
        request_was_slew_limited: slew_limited,
        candidate_age_ticks: age,
        remaining_fresh_ticks: remaining,
        last_pressure: state.last_pressure,
        target: state.target,
        request: if executable { state.request } else { [0.0; N] },
        transition_count: state.transition_count,
        flags,
    }
}

fn slew<const N: usize>(
    state: &mut ViabilityRequestState<N>,
    config: ViabilityRequestConfig<N>,
) -> bool {
    let mut limited = false;
    for coordinate in 0..N {
        let delta = state.target[coordinate] - state.request[coordinate];
        let bounded = delta.clamp(
            -config.maximum_slew_per_tick[coordinate],
            config.maximum_slew_per_tick[coordinate],
        );
        limited |= bounded.to_bits() != delta.to_bits();
        state.request[coordinate] += bounded;
        if state.request[coordinate].abs() <= f64::EPSILON && state.target[coordinate] == 0.0 {
            state.request[coordinate] = 0.0;
        }
    }
    limited
}

/// Advance one explicit planner-request supervision tick.
///
/// `planner_update` distinguishes an intentionally slower planner cadence from
/// a failed planner event. When it is true, `pressure` must be finite. An
/// active update without `candidate_available` revokes the previous candidate
/// immediately. Between updates, a candidate may be held only through the
/// configured tick age. Contact evidence is fail-closed and revokes the
/// request without a slew tail.
#[allow(clippy::too_many_arguments)]
pub fn step_viability_request<const N: usize>(
    tick_sequence: u64,
    observation_exact: bool,
    planner_update: bool,
    pressure: f64,
    candidate_available: bool,
    candidate: [f64; N],
    config: ViabilityRequestConfig<N>,
    state: &mut ViabilityRequestState<N>,
) -> ViabilityRequestOutput<N> {
    if !valid_config(config) {
        return output(
            state,
            ViabilityRequestStatus::InvalidConfig,
            ViabilityRequestProvenance::Unavailable,
            false,
            false,
            0,
            0,
            VIABILITY_REQUEST_INPUT_REJECTED,
        );
    }
    if state.tick_initialized && tick_sequence <= state.last_tick_sequence {
        return output(
            state,
            ViabilityRequestStatus::RejectedSequence,
            ViabilityRequestProvenance::Unavailable,
            false,
            false,
            tick_sequence.saturating_sub(state.candidate_tick_sequence),
            0,
            VIABILITY_REQUEST_SEQUENCE_REJECTED,
        );
    }
    let input_invalid = (planner_update && (!pressure.is_finite() || pressure < 0.0))
        || (!planner_update && candidate_available)
        || (candidate_available
            && candidate.iter().enumerate().any(|(coordinate, value)| {
                !value.is_finite() || value.abs() > config.maximum_abs_request[coordinate]
            }));
    if input_invalid {
        let mut next = *state;
        next.tick_initialized = true;
        next.last_tick_sequence = tick_sequence;
        next.active = false;
        next.has_candidate = false;
        next.target = [0.0; N];
        next.request = [0.0; N];
        let flags = set_status(&mut next, ViabilityRequestStatus::RejectedInput)
            | VIABILITY_REQUEST_INPUT_REJECTED;
        *state = next;
        return output(
            state,
            state.status,
            ViabilityRequestProvenance::Unavailable,
            false,
            false,
            0,
            0,
            flags,
        );
    }

    let mut next = *state;
    next.tick_initialized = true;
    next.last_tick_sequence = tick_sequence;
    if !observation_exact {
        next.active = false;
        next.has_candidate = false;
        next.target = [0.0; N];
        next.request = [0.0; N];
        let flags = set_status(&mut next, ViabilityRequestStatus::RevokedEvidence)
            | VIABILITY_REQUEST_EVIDENCE_REVOKED;
        *state = next;
        return output(
            state,
            state.status,
            ViabilityRequestProvenance::Unavailable,
            false,
            false,
            0,
            0,
            flags,
        );
    }

    if planner_update {
        next.last_pressure = pressure;
        if next.active {
            if pressure <= config.release_pressure {
                next.active = false;
                next.has_candidate = false;
                next.target = [0.0; N];
            }
        } else if pressure >= config.activation_pressure {
            next.active = true;
        }
        if next.active {
            if candidate_available {
                next.has_candidate = true;
                next.candidate_tick_sequence = tick_sequence;
                next.target = candidate;
                let limited = slew(&mut next, config);
                let flags =
                    set_status(&mut next, ViabilityRequestStatus::Fresh) | VIABILITY_REQUEST_FRESH;
                *state = next;
                return output(
                    state,
                    state.status,
                    ViabilityRequestProvenance::Fresh,
                    true,
                    limited,
                    0,
                    config.maximum_candidate_age_ticks,
                    flags,
                );
            }
            next.has_candidate = false;
            next.target = [0.0; N];
            next.request = [0.0; N];
            let flags =
                set_status(&mut next, ViabilityRequestStatus::Expired) | VIABILITY_REQUEST_EXPIRED;
            *state = next;
            return output(
                state,
                state.status,
                ViabilityRequestProvenance::Unavailable,
                false,
                false,
                0,
                0,
                flags,
            );
        }

        let limited = slew(&mut next, config);
        let executable = next.request.iter().any(|value| *value != 0.0);
        let status = if executable {
            ViabilityRequestStatus::Releasing
        } else {
            ViabilityRequestStatus::Inactive
        };
        let flags = set_status(&mut next, status)
            | if executable {
                VIABILITY_REQUEST_RELEASING
            } else {
                0
            };
        *state = next;
        return output(
            state,
            state.status,
            if executable {
                ViabilityRequestProvenance::Releasing
            } else {
                ViabilityRequestProvenance::Unavailable
            },
            executable,
            limited,
            0,
            0,
            flags,
        );
    }

    if !next.active {
        let limited = slew(&mut next, config);
        let executable = next.request.iter().any(|value| *value != 0.0);
        let status = if executable {
            ViabilityRequestStatus::Releasing
        } else {
            ViabilityRequestStatus::Inactive
        };
        let flags = set_status(&mut next, status)
            | if executable {
                VIABILITY_REQUEST_RELEASING
            } else {
                0
            };
        *state = next;
        return output(
            state,
            state.status,
            if executable {
                ViabilityRequestProvenance::Releasing
            } else {
                ViabilityRequestProvenance::Unavailable
            },
            executable,
            limited,
            0,
            0,
            flags,
        );
    }
    if !next.has_candidate {
        next.target = [0.0; N];
        next.request = [0.0; N];
        let flags =
            set_status(&mut next, ViabilityRequestStatus::Expired) | VIABILITY_REQUEST_EXPIRED;
        *state = next;
        return output(
            state,
            state.status,
            ViabilityRequestProvenance::Unavailable,
            false,
            false,
            0,
            0,
            flags,
        );
    }
    let age = tick_sequence.saturating_sub(next.candidate_tick_sequence);
    if age > u64::from(config.maximum_candidate_age_ticks) {
        next.has_candidate = false;
        next.target = [0.0; N];
        next.request = [0.0; N];
        let flags =
            set_status(&mut next, ViabilityRequestStatus::Expired) | VIABILITY_REQUEST_EXPIRED;
        *state = next;
        return output(
            state,
            state.status,
            ViabilityRequestProvenance::Unavailable,
            false,
            false,
            age,
            0,
            flags,
        );
    }
    let limited = slew(&mut next, config);
    let remaining = config.maximum_candidate_age_ticks - age as u32;
    let flags = set_status(&mut next, ViabilityRequestStatus::Held) | VIABILITY_REQUEST_HELD;
    *state = next;
    output(
        state,
        state.status,
        ViabilityRequestProvenance::Held,
        true,
        limited,
        age,
        remaining,
        flags,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    type State = ViabilityRequestState<3>;

    fn config() -> ViabilityRequestConfig<3> {
        ViabilityRequestConfig {
            activation_pressure: 0.10,
            release_pressure: 0.05,
            maximum_candidate_age_ticks: 4,
            maximum_abs_request: [250.0, 250.0, 80.0],
            maximum_slew_per_tick: [40.0, 40.0, 20.0],
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn step(
        tick: u64,
        exact: bool,
        update: bool,
        pressure: f64,
        available: bool,
        candidate: [f64; 3],
        state: &mut State,
    ) -> ViabilityRequestOutput<3> {
        step_viability_request(
            tick,
            exact,
            update,
            pressure,
            available,
            candidate,
            config(),
            state,
        )
    }

    #[test]
    fn activation_hold_refresh_and_hysteretic_release_are_bounded() {
        let mut state = State::default();
        let fresh = step(1, true, true, 0.11, true, [100.0, -60.0, 30.0], &mut state);
        assert_eq!(fresh.status, ViabilityRequestStatus::Fresh);
        assert_eq!(fresh.request, [40.0, -40.0, 20.0]);
        assert!(fresh.request_was_slew_limited);
        let held = step(2, true, false, 0.0, false, [0.0; 3], &mut state);
        assert_eq!(held.status, ViabilityRequestStatus::Held);
        assert_eq!(held.request, [80.0, -60.0, 30.0]);
        assert_eq!(held.candidate_age_ticks, 1);
        let refreshed = step(5, true, true, 0.08, true, [100.0, 20.0, 0.0], &mut state);
        assert_eq!(refreshed.status, ViabilityRequestStatus::Fresh);
        assert!(refreshed.active);
        let releasing = step(6, true, true, 0.04, false, [0.0; 3], &mut state);
        assert_eq!(releasing.status, ViabilityRequestStatus::Releasing);
        assert!(!releasing.active);
        assert!(releasing.executable);
        let still_releasing = step(7, true, false, 0.0, false, [0.0; 3], &mut state);
        assert_eq!(still_releasing.status, ViabilityRequestStatus::Releasing);
        let inactive = step(8, true, false, 0.0, false, [0.0; 3], &mut state);
        assert_eq!(inactive.status, ViabilityRequestStatus::Inactive);
        assert!(!inactive.executable);
        assert_eq!(inactive.request, [0.0; 3]);
    }

    #[test]
    fn candidate_age_expires_without_a_slew_tail() {
        let mut state = State::default();
        step(10, true, true, 0.2, true, [80.0, 0.0, 0.0], &mut state);
        for tick in 11..=14 {
            assert!(step(tick, true, false, 0.0, false, [0.0; 3], &mut state).executable);
        }
        let expired = step(15, true, false, 0.0, false, [0.0; 3], &mut state);
        assert_eq!(expired.status, ViabilityRequestStatus::Expired);
        assert!(!expired.executable);
        assert_eq!(expired.request, [0.0; 3]);
    }

    #[test]
    fn evidence_and_failed_planner_update_revoke_immediately() {
        let mut state = State::default();
        step(1, true, true, 0.2, true, [80.0, 0.0, 0.0], &mut state);
        let evidence = step(2, false, false, 0.0, false, [0.0; 3], &mut state);
        assert_eq!(evidence.status, ViabilityRequestStatus::RevokedEvidence);
        assert_eq!(evidence.request, [0.0; 3]);
        step(3, true, true, 0.2, true, [80.0, 0.0, 0.0], &mut state);
        let failed = step(4, true, true, 0.2, false, [0.0; 3], &mut state);
        assert_eq!(failed.status, ViabilityRequestStatus::Expired);
        assert_eq!(failed.request, [0.0; 3]);
    }

    #[test]
    fn reordered_tick_is_atomic_and_invalid_candidate_fails_closed() {
        let mut state = State::default();
        step(5, true, true, 0.2, true, [80.0, 0.0, 0.0], &mut state);
        let before = state;
        let duplicate = step(5, true, false, 0.0, false, [0.0; 3], &mut state);
        assert_eq!(duplicate.status, ViabilityRequestStatus::RejectedSequence);
        assert!(!duplicate.executable);
        assert_ne!(duplicate.flags & VIABILITY_REQUEST_SEQUENCE_REJECTED, 0);
        assert_eq!(state, before);
        let invalid = step(6, true, true, 0.2, true, [251.0, 0.0, 0.0], &mut state);
        assert_eq!(invalid.status, ViabilityRequestStatus::RejectedInput);
        assert!(!invalid.executable);
        assert_eq!(invalid.request, [0.0; 3]);
    }
}
