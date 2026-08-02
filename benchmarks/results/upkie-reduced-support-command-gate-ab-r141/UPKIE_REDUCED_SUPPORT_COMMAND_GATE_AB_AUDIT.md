# Bonesaw reduced-support command gate A/B · r141

> Deployment **REJECTED**. The candidate consumes r139 exact contact evidence; simulator truth is not a hardware estimator.

## Outcome

The frozen 20-case r133 consequence matrix is rerun as baseline/current-live freshness versus causal measured-contact hard masks whose reduced-support WBC result remains diagnostic-only. The prior admitted double-support command expires through r137 until exact double support is re-established. Promotion requires every baseline-qualified row, no new or earlier fall, no numeric fault, non-increased non-admission, exact replay, and zero Rust allocation.

| case | baseline | candidate | boundary Δ s | nonadmitted | min contacts | transitions | mask mismatch ticks | replay |
|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | — | 5 → 2 | 0 | 2 | 2 | YES |
| forward_2n | RECOVERED | RECOVERED | — | 5 → 4 | 0 | 2 | 2 | YES |
| forward_4n_reference | RECOVERED | RECOVERED | — | 5 → 5 | 0 | 2 | 2 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.365s | -3.455 | 76 → 46 | 0 | 4 | 7 | YES |
| backward_2n | RECOVERED | RECOVERED | — | 5 → 3 | 0 | 2 | 2 | YES |
| backward_4n | RECOVERED | RECOVERED | — | 5 → 8 | 0 | 2 | 2 | YES |
| left_1n | FALL 2.470s | FALL 1.810s | -0.660 | 64 → 70 | 0 | 11 | 19 | YES |
| left_2n | FALL 2.350s | FALL 1.415s | -0.935 | 88 → 61 | 0 | 4 | 17 | YES |
| left_4n | FALL 2.610s | FALL 1.685s | -0.925 | 133 → 60 | 0 | 4 | 10 | YES |
| right_2n | FALL 2.015s | FALL 1.670s | -0.345 | 19 → 67 | 0 | 7 | 9 | YES |
| right_4n | FALL 2.415s | FALL 1.395s | -1.020 | 72 → 64 | 0 | 6 | 14 | YES |
| diagonal_4n | FALL 2.125s | FALL 1.330s | -0.795 | 65 → 63 | 0 | 7 | 17 | YES |
| up_4n | RECOVERED | RECOVERED | — | 5 → 2 | 0 | 2 | 2 | YES |
| down_4n | RECOVERED | RECOVERED | — | 5 → 2 | 0 | 2 | 2 | YES |
| handle_forward_4n | FALL 4.630s | FALL 1.385s | -3.245 | 96 → 36 | 0 | 4 | 2 | YES |
| short_8n_50ms | RECOVERED | RECOVERED | — | 5 → 9 | 0 | 2 | 2 | YES |
| long_2n_200ms | RECOVERED | RECOVERED | — | 5 → 8 | 0 | 2 | 2 | YES |
| forward_2n_three_pulses | RECOVERED | RECOVERED | — | 5 → 5 | 0 | 2 | 2 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | — | 9 → 4 | 0 | 2 | 2 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.315s | -1.295 | 39 → 56 | 0 | 4 | 11 | YES |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| no_new_numeric_fault | PASS |
| green_qualification_preserved | FAIL |
| no_new_fall | PASS |
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
