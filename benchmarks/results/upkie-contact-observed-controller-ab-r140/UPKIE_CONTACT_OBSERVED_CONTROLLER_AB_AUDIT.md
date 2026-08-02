# Bonesaw measured-contact controller A/B · r140

> Deployment **REJECTED**. The candidate consumes r139 exact contact evidence; simulator truth is not a hardware estimator.

## Outcome

The frozen 20-case r133 consequence matrix is rerun as baseline/current-live freshness versus causal measured-contact hard masks, with an exact candidate replay. A new contact requires three exact samples; exact absence removes its hard row immediately; rejected WBC candidates remain non-executable and prior torque still expires through r137. Promotion requires every baseline-qualified row, no new or earlier fall, no numeric fault, non-increased non-admission, exact replay, and zero Rust allocation.

| case | baseline | candidate | boundary Δ s | nonadmitted | min contacts | transitions | mask mismatch ticks | replay |
|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | FALL 3.395s | — | 5 → 3 | 0 | 10 | 10 | YES |
| forward_2n | RECOVERED | FALL 3.680s | — | 5 → 1 | 0 | 6 | 8 | YES |
| forward_4n_reference | RECOVERED | FALL 2.865s | — | 5 → 3 | 0 | 18 | 18 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | -3.590 | 76 → 0 | 0 | 8 | 9 | YES |
| backward_2n | RECOVERED | FALL 3.420s | — | 5 → 2 | 0 | 10 | 11 | YES |
| backward_4n | RECOVERED | FALL 2.730s | — | 5 → 1 | 0 | 10 | 12 | YES |
| left_1n | FALL 2.470s | FALL 2.295s | -0.175 | 64 → 8 | 0 | 16 | 17 | YES |
| left_2n | FALL 2.350s | FALL 2.535s | +0.185 | 88 → 6 | 0 | 8 | 7 | YES |
| left_4n | FALL 2.610s | FALL 1.850s | -0.760 | 133 → 1 | 0 | 6 | 7 | YES |
| right_2n | FALL 2.015s | FALL 2.030s | +0.015 | 19 → 4 | 0 | 9 | 9 | YES |
| right_4n | FALL 2.415s | FALL 2.195s | -0.220 | 72 → 0 | 0 | 11 | 12 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.605s | +0.480 | 65 → 1 | 0 | 9 | 12 | YES |
| up_4n | RECOVERED | FALL 4.215s | — | 5 → 1 | 0 | 19 | 23 | YES |
| down_4n | RECOVERED | FALL 3.450s | — | 5 → 6 | 0 | 9 | 11 | YES |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | -2.585 | 96 → 0 | 0 | 14 | 19 | YES |
| short_8n_50ms | RECOVERED | FALL 1.785s | — | 5 → 0 | 0 | 10 | 15 | YES |
| long_2n_200ms | RECOVERED | FALL 3.155s | — | 5 → 4 | 0 | 5 | 4 | YES |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | — | 5 → 2 | 0 | 6 | 6 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | — | 9 → 0 | 0 | 2 | 2 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | -0.895 | 39 → 0 | 0 | 10 | 14 | YES |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| no_new_numeric_fault | PASS |
| green_qualification_preserved | FAIL |
| no_new_fall | FAIL |
| no_earlier_fall_boundary | FAIL |
| nonadmission_not_increased | PASS |
| contact_transition_discriminates | PASS |
| candidate_replay_exact | PASS |
| zero_rust_allocation | PASS |
| finite | PASS |

## Contract

- Baseline and candidate share plant, capture policy, WBC, effort limits, fall-safe command lease, disturbances, and first-boundary scoring. Only contact observation/admission differs.
- Exact absence cannot retain a hard rolling row. Debounced mode and hard eligibility remain separate.
- A passing simulator A/B may promote the software boundary for the toy only; hardware still needs calibrated sensing, delay/noise/dropout, and source-authentication evidence.
