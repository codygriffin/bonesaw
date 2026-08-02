# Bonesaw bounded viability-request supervisor · r148

> Evaluation **PASS**. No policy, WBC, simulator, integration, or wall clock participates in the request decision. Python authors explicit rows; Rust owns hysteresis, freshness, slew, sequence, bounds, and revocation.

## Outcome

A planner candidate is not an executable command. The supervisor activates only above the declared pressure, slews all three coordinates, holds only inside a four-tick freshness window, releases hysteretically, and emits exact zero on stale age, failed refresh, inexact evidence, invalid bounds, or rejected sequence. Downstream WBC admission remains mandatory every tick.

| scenario | final status | active | executable | age | request | replay |
|---|---|---|---|---|---|---|
| hysteresis_release | Inactive | NO | NO | 0 | +0/+0/+0 | YES |
| expiry | Expired | YES | NO | 5 | +0/+0/+0 | YES |
| evidence_revocation | RevokedEvidence | NO | NO | 0 | +0/+0/+0 | YES |
| failed_refresh | Expired | YES | NO | 0 | +0/+0/+0 | YES |
| bounded_input | RejectedInput | NO | NO | 0 | +0/+0/+0 | YES |
| duplicate_tick | Held | YES | YES | 1 | +80/+0/+0 | YES |

## Gates

| gate | result |
|---|---|
| fresh_request_is_bounded_and_slewed | PASS |
| held_request_reaches_bounded_target | PASS |
| release_is_slewed_then_inactive | PASS |
| age_expiry_has_no_tail | PASS |
| inexact_evidence_revokes_immediately | PASS |
| failed_active_refresh_revokes | PASS |
| out_of_bounds_candidate_rejected | PASS |
| duplicate_tick_is_atomic | PASS |
| exact_replay | PASS |
| finite_outputs | PASS |

## Host call timing

| steps | p50 ns | p95 ns | p99 ns | p99.9 ns | max ns |
|---|---|---|---|---|---|
| 20000 | 571.0 | 591.0 | 611.0 | 751.0010000000038 | 15048 |

The Python-call timing includes binding overhead. The binding separately compares Rust allocation counters around every supervisor step and fails the call if the Rust hot path allocates.
