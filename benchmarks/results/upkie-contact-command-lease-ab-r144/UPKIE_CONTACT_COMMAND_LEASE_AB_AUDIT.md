# Bonesaw bounded contact-command lease A/B · r144

> Physical deployment **REJECTED**. The r143 lease is configured for 5 ticks (25 ms); exact MuJoCo contact remains a simulator source, not a hardware estimator.

## Outcome

The frozen 20-case r133 consequence matrix compares current-live fixed double-support control with r139 exact contact evidence, diagnostic-only reduced-support solves, and the r143 bounded prior-command lease. The lease itself is enforced end to end: no torque survives a non-executable lease, exact replay passes, and Rust timed allocations remain zero. Physical promotion remains independently gated on every green recovery and every existing adverse boundary.

| case | baseline | candidate | boundary Δ s | leased ticks | max age | post-expiry torque | replay |
|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | — | 0 | 0 | 0.0e+00 | YES |
| forward_2n | RECOVERED | RECOVERED | — | 2 | 1 | 0.0e+00 | YES |
| forward_4n_reference | RECOVERED | RECOVERED | — | 3 | 1 | 0.0e+00 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.315s | -3.505 | 8 | 6 | 0.0e+00 | YES |
| backward_2n | RECOVERED | RECOVERED | — | 1 | 1 | 0.0e+00 | YES |
| backward_4n | RECOVERED | RECOVERED | — | 6 | 1 | 0.0e+00 | YES |
| left_1n | FALL 2.470s | FALL 1.805s | -0.665 | 8 | 6 | 0.0e+00 | YES |
| left_2n | FALL 2.350s | FALL 2.485s | +0.135 | 23 | 6 | 0.0e+00 | YES |
| left_4n | FALL 2.610s | FALL 1.685s | -0.925 | 10 | 6 | 0.0e+00 | YES |
| right_2n | FALL 2.015s | FALL 2.040s | +0.025 | 7 | 6 | 0.0e+00 | YES |
| right_4n | FALL 2.415s | FALL 2.190s | -0.225 | 7 | 6 | 0.0e+00 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.060s | -0.065 | 8 | 6 | 0.0e+00 | YES |
| up_4n | RECOVERED | RECOVERED | — | 0 | 0 | 0.0e+00 | YES |
| down_4n | RECOVERED | RECOVERED | — | 0 | 0 | 0.0e+00 | YES |
| handle_forward_4n | FALL 4.630s | FALL 1.395s | -3.235 | 9 | 6 | 0.0e+00 | YES |
| short_8n_50ms | RECOVERED | RECOVERED | — | 7 | 1 | 0.0e+00 | YES |
| long_2n_200ms | RECOVERED | RECOVERED | — | 6 | 1 | 0.0e+00 | YES |
| forward_2n_three_pulses | RECOVERED | RECOVERED | — | 3 | 1 | 0.0e+00 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | — | 2 | 1 | 0.0e+00 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.280s | -1.330 | 6 | 6 | 0.0e+00 | YES |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| lease_transition_exercised | PASS |
| bounded_command_age | PASS |
| no_torque_after_nonexecutable_lease | PASS |
| no_new_numeric_fault | PASS |
| green_qualification_preserved | FAIL |
| no_new_fall | PASS |
| no_earlier_fall_boundary | FAIL |
| candidate_replay_exact | PASS |
| zero_rust_allocation | PASS |
| finite | PASS |

## Boundary

- R143 admits only bounded command lifetime. This A/B determines whether that mechanism may drive the plant; it does not alter the lease contract when physical promotion fails.
- Reduced-support WBC results remain diagnostic-only. A leased torque retains the prior authoring mask and emits no current-support contact-force witness.
- The next action must be independently solved for the observed support state or prepared before contact loss; extending this hold horizon cannot be called recovery.
