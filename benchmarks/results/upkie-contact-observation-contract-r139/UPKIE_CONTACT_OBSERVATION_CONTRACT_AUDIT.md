# Bonesaw causal contact-observation contract · r139

> Evaluation **PASS**. No WBC, policy, simulator, integration, or clock participates; Python supplies immutable observation rows and Rust owns every state transition.

## Outcome

The contract separates exact raw contact, debounced support mode, and hard-row eligibility. A new contact needs three consecutive exact samples. Exact absence removes its hard row immediately, while the stable mode changes after two samples. Missing, held, future, stale, uncertain, wrong-source, duplicate, and time-reordered evidence never authors a hard row. Left-only and right-only transitions are mirrored and exact replay passes.

| case | final raw | final mode | final hard | transitions | replay |
|---|---|---|---|---|---|
| activate_double | 11 | 11 | 11 | 2 | YES |
| left_only | 10 | 10 | 10 | 3 | YES |
| right_only | 01 | 01 | 01 | 3 | YES |
| zero_support | 00 | 00 | 00 | 4 | YES |
| loss_chatter | 11 | 11 | 11 | 2 | YES |
| activation_chatter | 10 | 10 | 10 | 1 | YES |

## Fault admission

| fault | status | provenance | hard | state atomic |
|---|---|---|---|---|
| future | RejectedFuture | Held | 00 | YES |
| stale | RejectedStale | Unavailable | 00 | YES |
| uncertain | RejectedUncertain | Held | 00 | YES |
| wrong_source | RejectedSource | Held | 00 | YES |
| duplicate_sequence | RejectedSequence | Held | 00 | YES |
| old_timestamp | RejectedTimestamp | Held | 00 | YES |
| missing_held | Missing | Held | 00 | YES |
| missing_expired | Missing | Unavailable | 00 | YES |

## Gates

| gate | result |
|---|---|
| three_sample_activation | PASS |
| absence_removes_hard_row_immediately | PASS |
| two_sample_mode_deactivation | PASS |
| mirrored_single_contact | PASS |
| loss_chatter_does_not_change_mode | PASS |
| activation_chatter_requires_consecutive_evidence | PASS |
| faults_typed_atomic_and_nonhard | PASS |
| missing_provenance_expires | PASS |
| exact_replay | PASS |

## Admission boundary

- The caller owns control time and observation clock mapping. Rust validates mapped time, age, source identity, sequence, and synchronization uncertainty.
- Held mode state remains visible but is never hard-constraint eligible. A rejected observation is atomic.
- Debounce state is not contact estimation. Live promotion still requires a calibrated source and a controller A/B for each support mode.
