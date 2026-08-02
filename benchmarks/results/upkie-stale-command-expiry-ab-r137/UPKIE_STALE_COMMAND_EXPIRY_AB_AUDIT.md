# Bonesaw Upkie stale-command expiry A/B · r137

> Evaluation **PASS**. Live promotion covers command freshness only; it is not a recovery claim.

## Outcome

The deployed candidate leaves valid primary objectives untouched. When the current WBC rejects, Rust grants the last admitted torque a five-tick lease and fades it to zero by tick twelve. Nominal and the qualified 4 N sagittal recovery are bit-exact. Every frozen adverse row reaches the first boundary no earlier, and stale-command age falls in every row. Terminal energy is reported but is deliberately not an admission gate: expiring stale torque is a freshness guarantee, not a universal energy controller.

| case | baseline | candidate | boundary Δ s | max stale age | min fresh authority | terminal KE J | baseline exact |
|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | — | 5 → 5 | 1.000 | 0.0000 → 0.0000 | YES |
| forward_4n | RECOVERED | RECOVERED | — | 5 → 5 | 1.000 | 0.0000 → 0.0000 | YES |
| left_1n | FALL 2.260s | FALL 2.470s | +0.210 | 48 → 17 | 0.000 | 2.7147 → 34.3496 | NO |
| left_2n | FALL 1.985s | FALL 2.350s | +0.365 | 26 → 22 | 0.000 | 86.5592 → 37.4664 | NO |
| right_2n | FALL 1.990s | FALL 2.015s | +0.025 | 20 → 11 | 0.055 | 4.1332 → 0.8859 | NO |
| forward_6n_overload | FALL 4.615s | FALL 4.820s | +0.205 | 41 → 28 | 0.000 | 32.9364 → 26.1317 | NO |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| green_recovery_preserved | PASS |
| green_semantics_exact | PASS |
| no_earlier_adverse_boundary | PASS |
| stale_command_age_reduced | PASS |
| expiry_engages | PASS |
| candidate_replay_exact | PASS |
| zero_rust_allocation | PASS |
| finite_metrics | PASS |

## Contract

- A `MaxIterations` candidate never executes. Only a previously admitted command may use the bounded lease.
- Startup has an explicit five-tick lease before the first admitted command; it does not count as stale replay.
- The r136 physical-risk damping blend remains rejected because the overload boundary regressed by 3.120 s.
- Rust owns lease state, freshness authority, reason flags, and allocation-free state transition. Python owns MuJoCo and retained A/B evidence.
