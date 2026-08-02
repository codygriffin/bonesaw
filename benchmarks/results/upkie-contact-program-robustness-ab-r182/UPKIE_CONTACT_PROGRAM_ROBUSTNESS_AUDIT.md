# Bonesaw contact-program observation robustness A/B · r182

> Mechanism **PASS** · robustness profile **REJECTED**. Python owns deterministic sensor perturbation and MuJoCo; Rust owns timestamp/provenance admission, debounce, current-support WBC proof, and program authority.

## Result

| case | profile | exact | perturbed | fall Δ s | unavailable | mask mismatch | withheld | >5 ms |
|---|---|---|---|---|---|---|---|---|
| nominal | delay_5ms | RECOVERED | RECOVERED | — | 0 | 0 | 1 | 0 |
| nominal | delay_20ms | RECOVERED | RECOVERED | — | 0 | 0 | 5 | 0 |
| nominal | dropout_5ms_per_250ms | RECOVERED | RECOVERED | — | 24 | 0 | 24 | 0 |
| nominal | dropout_10ms_per_500ms | RECOVERED | RECOVERED | — | 24 | 0 | 24 | 0 |
| nominal | left_bit_chatter_5ms_per_250ms | RECOVERED | RECOVERED | — | 0 | 24 | 1 | 0 |
| nominal | right_bit_chatter_5ms_per_250ms | RECOVERED | RECOVERED | — | 0 | 24 | 1 | 0 |
| forward_4n_reference | delay_5ms | RECOVERED | RECOVERED | — | 0 | 10 | 1 | 0 |
| forward_4n_reference | delay_20ms | RECOVERED | RECOVERED | — | 0 | 10 | 5 | 0 |
| forward_4n_reference | dropout_5ms_per_250ms | RECOVERED | RECOVERED | — | 24 | 0 | 24 | 0 |
| forward_4n_reference | dropout_10ms_per_500ms | RECOVERED | RECOVERED | — | 24 | 0 | 24 | 0 |
| forward_4n_reference | left_bit_chatter_5ms_per_250ms | RECOVERED | RECOVERED | — | 0 | 24 | 1 | 0 |
| forward_4n_reference | right_bit_chatter_5ms_per_250ms | RECOVERED | RECOVERED | — | 0 | 24 | 1 | 0 |
| backward_4n | delay_5ms | RECOVERED | RECOVERED | — | 0 | 8 | 1 | 0 |
| backward_4n | delay_20ms | RECOVERED | RECOVERED | — | 0 | 8 | 5 | 0 |
| backward_4n | dropout_5ms_per_250ms | RECOVERED | RECOVERED | — | 24 | 0 | 24 | 0 |
| backward_4n | dropout_10ms_per_500ms | RECOVERED | RECOVERED | — | 24 | 0 | 24 | 0 |
| backward_4n | left_bit_chatter_5ms_per_250ms | RECOVERED | RECOVERED | — | 0 | 24 | 1 | 0 |
| backward_4n | right_bit_chatter_5ms_per_250ms | RECOVERED | RECOVERED | — | 0 | 24 | 1 | 0 |
| left_1n | delay_5ms | FALL 2.470s | FALL 2.545s | +0.075 | 0 | 60 | 1 | 4 |
| left_1n | delay_20ms | FALL 2.470s | FALL 2.760s | +0.290 | 0 | 102 | 5 | 3 |
| left_1n | dropout_5ms_per_250ms | FALL 2.470s | FALL 1.920s | -0.550 | 8 | 0 | 10 | 0 |
| left_1n | dropout_10ms_per_500ms | FALL 2.470s | FALL 3.340s | +0.870 | 14 | 0 | 15 | 1 |
| left_1n | left_bit_chatter_5ms_per_250ms | FALL 2.470s | FALL 2.545s | +0.075 | 0 | 11 | 1 | 4 |
| left_1n | right_bit_chatter_5ms_per_250ms | FALL 2.470s | FALL 2.545s | +0.075 | 0 | 11 | 1 | 2 |
| right_1n_mirror | delay_5ms | FALL 2.520s | FALL 2.760s | +0.240 | 0 | 104 | 1 | 8 |
| right_1n_mirror | delay_20ms | FALL 2.520s | FALL 2.530s | +0.010 | 0 | 53 | 5 | 2 |
| right_1n_mirror | dropout_5ms_per_250ms | FALL 2.520s | FALL 2.275s | -0.245 | 10 | 0 | 10 | 0 |
| right_1n_mirror | dropout_10ms_per_500ms | FALL 2.520s | FALL 2.975s | +0.455 | 12 | 0 | 13 | 0 |
| right_1n_mirror | left_bit_chatter_5ms_per_250ms | FALL 2.520s | FALL 2.760s | +0.240 | 0 | 12 | 1 | 8 |
| right_1n_mirror | right_bit_chatter_5ms_per_250ms | FALL 2.520s | FALL 2.760s | +0.240 | 0 | 12 | 1 | 7 |
| handle_forward_4n | delay_5ms | FALL 4.630s | FALL 5.950s | +1.320 | 0 | 71 | 1 | 4 |
| handle_forward_4n | delay_20ms | FALL 4.630s | FALL 3.005s | -1.625 | 0 | 80 | 5 | 0 |
| handle_forward_4n | dropout_5ms_per_250ms | FALL 4.630s | FALL 4.155s | -0.475 | 17 | 0 | 18 | 1 |
| handle_forward_4n | dropout_10ms_per_500ms | FALL 4.630s | FALL 4.305s | -0.325 | 18 | 0 | 18 | 3 |
| handle_forward_4n | left_bit_chatter_5ms_per_250ms | FALL 4.630s | FALL 5.950s | +1.320 | 0 | 24 | 1 | 1 |
| handle_forward_4n | right_bit_chatter_5ms_per_250ms | FALL 4.630s | FALL 5.950s | +1.320 | 0 | 24 | 1 | 2 |
| forward_4n_friction_0p03 | delay_5ms | FALL 1.610s | FALL 1.675s | +0.065 | 0 | 36 | 1 | 0 |
| forward_4n_friction_0p03 | delay_20ms | FALL 1.610s | FALL 0.625s | -0.985 | 0 | 53 | 4 | 2 |
| forward_4n_friction_0p03 | dropout_5ms_per_250ms | FALL 1.610s | FALL 1.635s | +0.025 | 7 | 0 | 7 | 0 |
| forward_4n_friction_0p03 | dropout_10ms_per_500ms | FALL 1.610s | FALL 0.690s | -0.920 | 4 | 0 | 4 | 0 |
| forward_4n_friction_0p03 | left_bit_chatter_5ms_per_250ms | FALL 1.610s | FALL 1.675s | +0.065 | 0 | 7 | 1 | 0 |
| forward_4n_friction_0p03 | right_bit_chatter_5ms_per_250ms | FALL 1.610s | FALL 1.675s | +0.065 | 0 | 7 | 1 | 0 |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| exact_replay | PASS |
| dropout_exercised | PASS |
| dropout_is_fail_closed | PASS |
| inclusive_20ms_age_is_exercised | PASS |
| bit_chatter_and_both_sides_exercised | PASS |
| finite_without_numeric_fault | PASS |
| zero_rust_allocation_and_python_gc | PASS |

## Robustness gates

| gate | result |
|---|---|
| no_exact_green_case_becomes_a_fall | PASS |
| no_exact_fall_boundary_moves_earlier | FAIL |
| zero_5ms_loop_overruns | FAIL |

## Contract

- Delay profiles retain the sample mask and expose its acquisition age to Rust. The 20 ms row exercises the configured inclusive age boundary; it does not pretend delayed contact is current truth.
- Dropout profiles mark the observation unavailable. With a zero-tick command lease, unavailable evidence must produce withheld authority and exactly zero torque on that tick.
- Bit-chatter profiles are accepted sensor claims with deliberately wrong left or right bits. Their consequence measures sensitivity; software cannot infer that an authenticated but incorrect sensor bit is physically false.
- The mirrored ±1 N rows and left/right bit profiles expose both contact directions without averaging them into a symmetry claim.
- New falls from exact-green cases: **0** (none). Earlier exact-fall boundaries: **7** (left_1n/dropout_5ms_per_250ms, right_1n_mirror/dropout_5ms_per_250ms, handle_forward_4n/delay_20ms, handle_forward_4n/dropout_5ms_per_250ms, handle_forward_4n/dropout_10ms_per_500ms, forward_4n_friction_0p03/delay_20ms, forward_4n_friction_0p03/dropout_10ms_per_500ms).
