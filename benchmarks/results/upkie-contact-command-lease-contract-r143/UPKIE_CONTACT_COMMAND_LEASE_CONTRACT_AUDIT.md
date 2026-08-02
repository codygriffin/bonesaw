# Bonesaw bounded contact-command lease · r143

> Evaluation **PASS**. No WBC, policy, simulator, integration, or clock participates; Python supplies immutable rows and Rust owns command bytes, authoring support, tick age, and revocation.

## Outcome

An already admitted six-actuator command can cross an exact contact-mask edge for a fixed number of explicit ticks. The lease is mirrored for left-only and right-only support, covers zero support without claiming recovery, emits zero after expiry, and cannot resurrect after missing evidence or support return. This closes a command-lifetime contract only; it does not admit the command as physically safe.

| case | final status | provenance | age ticks | executable | replay |
|---|---|---|---|---|---|
| left_only | Expired | Unavailable | 3 | NO | YES |
| right_only | Expired | Unavailable | 3 | NO | YES |
| zero_support | Expired | Unavailable | 3 | NO | YES |
| support_return | Unavailable | Unavailable | 0 | NO | YES |
| missing_evidence | Unavailable | Unavailable | 0 | NO | YES |
| zero_hold | Expired | Unavailable | 1 | NO | YES |
| invalid_command | RejectedCommand | Unavailable | 0 | NO | YES |

## Gates

| gate | result |
|---|---|
| fresh_command_exact | PASS |
| left_right_zero_transitions_mirrored | PASS |
| bounded_expiry_zeroes_command | PASS |
| support_return_requires_fresh_admission | PASS |
| nonexact_evidence_revokes_without_resurrection | PASS |
| zero_hold_never_replays | PASS |
| invalid_command_nonexecutable | PASS |
| duplicate_tick_atomic | PASS |
| exact_replay | PASS |
| finite_outputs | PASS |

## Admission boundary

- Fresh commands must already be admitted by the caller against exact evidence; the lease never solves or approves one.
- Non-exact evidence, invalid commands, returned authoring support, and age expiry produce no executable command.
- A physical A/B must still choose the hold horizon and prove no earlier fall, no lost green recovery, and no unbounded stale execution.
