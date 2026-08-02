# Bonesaw observation-loss duration-oracle ablation · r187

> Mechanism **PASS** · duration-specific consequence **EXISTS** · one gain across both burst classes **REJECTED** · synchronous profile **REJECTED**.

## Gain decisions

| profile | decision | new green falls | earlier falls | worst Δ s |
|---|---|---|---|---|
| drop5_g000 | REJECTED | 0 | 3 | -1.245 |
| drop5_g025 | REJECTED | 0 | 1 | -0.930 |
| drop5_g045 | REJECTED | 1 | 1 | -0.315 |
| drop5_g050 | REJECTED | 1 | 2 | -0.575 |
| drop5_g075 | REJECTED | 0 | 2 | -1.780 |
| drop5_g095 | REJECTED | 0 | 1 | -0.105 |
| drop5_g100 | REJECTED | 0 | 1 | -0.040 |
| drop10_g000 | REJECTED | 0 | 2 | -0.155 |
| drop10_g025 | REJECTED | 0 | 2 | -1.415 |
| drop10_g045 | ADMITTED | 0 | 0 | — |
| drop10_g050 | REJECTED | 0 | 2 | -0.010 |
| drop10_g075 | REJECTED | 0 | 3 | -1.715 |
| drop10_g095 | ADMITTED | 0 | 0 | — |
| drop10_g100 | REJECTED | 0 | 2 | -1.655 |

## Plant rows

| case | profile | outcome | fall Δ s | gain Q15 | typed holds |
|---|---|---|---|---|---|
| nominal | drop5_g000 | RECOVERED | — | 0 | 0 |
| nominal | drop5_g025 | RECOVERED | — | 8192 | 23 |
| nominal | drop5_g045 | RECOVERED | — | 14746 | 23 |
| nominal | drop5_g050 | RECOVERED | — | 16384 | 23 |
| nominal | drop5_g075 | RECOVERED | — | 24576 | 23 |
| nominal | drop5_g095 | RECOVERED | — | 31130 | 23 |
| nominal | drop5_g100 | RECOVERED | — | 32768 | 23 |
| nominal | drop10_g000 | RECOVERED | — | 0 | 0 |
| nominal | drop10_g025 | RECOVERED | — | 8192 | 11 |
| nominal | drop10_g045 | RECOVERED | — | 14746 | 11 |
| nominal | drop10_g050 | RECOVERED | — | 16384 | 11 |
| nominal | drop10_g075 | RECOVERED | — | 24576 | 11 |
| nominal | drop10_g095 | RECOVERED | — | 31130 | 11 |
| nominal | drop10_g100 | RECOVERED | — | 32768 | 11 |
| forward_4n_reference | drop5_g000 | RECOVERED | — | 0 | 0 |
| forward_4n_reference | drop5_g025 | RECOVERED | — | 8192 | 23 |
| forward_4n_reference | drop5_g045 | RECOVERED | — | 14746 | 23 |
| forward_4n_reference | drop5_g050 | FALL 2.495s | — | 16384 | 9 |
| forward_4n_reference | drop5_g075 | RECOVERED | — | 24576 | 23 |
| forward_4n_reference | drop5_g095 | RECOVERED | — | 31130 | 23 |
| forward_4n_reference | drop5_g100 | RECOVERED | — | 32768 | 23 |
| forward_4n_reference | drop10_g000 | RECOVERED | — | 0 | 0 |
| forward_4n_reference | drop10_g025 | RECOVERED | — | 8192 | 11 |
| forward_4n_reference | drop10_g045 | RECOVERED | — | 14746 | 11 |
| forward_4n_reference | drop10_g050 | RECOVERED | — | 16384 | 11 |
| forward_4n_reference | drop10_g075 | RECOVERED | — | 24576 | 11 |
| forward_4n_reference | drop10_g095 | RECOVERED | — | 31130 | 11 |
| forward_4n_reference | drop10_g100 | RECOVERED | — | 32768 | 11 |
| backward_4n | drop5_g000 | RECOVERED | — | 0 | 0 |
| backward_4n | drop5_g025 | RECOVERED | — | 8192 | 23 |
| backward_4n | drop5_g045 | FALL 3.435s | — | 14746 | 13 |
| backward_4n | drop5_g050 | RECOVERED | — | 16384 | 23 |
| backward_4n | drop5_g075 | RECOVERED | — | 24576 | 23 |
| backward_4n | drop5_g095 | RECOVERED | — | 31130 | 23 |
| backward_4n | drop5_g100 | RECOVERED | — | 32768 | 23 |
| backward_4n | drop10_g000 | RECOVERED | — | 0 | 0 |
| backward_4n | drop10_g025 | RECOVERED | — | 8192 | 11 |
| backward_4n | drop10_g045 | RECOVERED | — | 14746 | 11 |
| backward_4n | drop10_g050 | RECOVERED | — | 16384 | 11 |
| backward_4n | drop10_g075 | RECOVERED | — | 24576 | 11 |
| backward_4n | drop10_g095 | RECOVERED | — | 31130 | 11 |
| backward_4n | drop10_g100 | RECOVERED | — | 32768 | 11 |
| left_1n | drop5_g000 | FALL 1.785s | -0.685 | 0 | 0 |
| left_1n | drop5_g025 | FALL 1.540s | -0.930 | 8192 | 6 |
| left_1n | drop5_g045 | FALL 2.960s | +0.490 | 14746 | 11 |
| left_1n | drop5_g050 | FALL 2.325s | -0.145 | 16384 | 9 |
| left_1n | drop5_g075 | FALL 2.570s | +0.100 | 24576 | 10 |
| left_1n | drop5_g095 | FALL 2.365s | -0.105 | 31130 | 9 |
| left_1n | drop5_g100 | FALL 2.430s | -0.040 | 32768 | 9 |
| left_1n | drop10_g000 | FALL 2.725s | +0.255 | 0 | 0 |
| left_1n | drop10_g025 | FALL 1.715s | -0.755 | 8192 | 3 |
| left_1n | drop10_g045 | FALL 2.900s | +0.430 | 14746 | 5 |
| left_1n | drop10_g050 | FALL 2.475s | +0.005 | 16384 | 4 |
| left_1n | drop10_g075 | FALL 2.220s | -0.250 | 24576 | 4 |
| left_1n | drop10_g095 | FALL 2.575s | +0.105 | 31130 | 5 |
| left_1n | drop10_g100 | FALL 4.175s | +1.705 | 32768 | 8 |
| right_1n_mirror | drop5_g000 | FALL 2.170s | -0.350 | 0 | 0 |
| right_1n_mirror | drop5_g025 | FALL 3.065s | +0.545 | 8192 | 12 |
| right_1n_mirror | drop5_g045 | FALL 4.025s | +1.505 | 14746 | 16 |
| right_1n_mirror | drop5_g050 | FALL 3.340s | +0.820 | 16384 | 13 |
| right_1n_mirror | drop5_g075 | FALL 1.560s | -0.960 | 24576 | 6 |
| right_1n_mirror | drop5_g095 | FALL 2.850s | +0.330 | 31130 | 11 |
| right_1n_mirror | drop5_g100 | FALL 3.035s | +0.515 | 32768 | 12 |
| right_1n_mirror | drop10_g000 | FALL 2.365s | -0.155 | 0 | 0 |
| right_1n_mirror | drop10_g025 | FALL 3.415s | +0.895 | 8192 | 6 |
| right_1n_mirror | drop10_g045 | FALL 2.675s | +0.155 | 14746 | 5 |
| right_1n_mirror | drop10_g050 | FALL 2.510s | -0.010 | 16384 | 5 |
| right_1n_mirror | drop10_g075 | FALL 2.000s | -0.520 | 24576 | 3 |
| right_1n_mirror | drop10_g095 | FALL 2.620s | +0.100 | 31130 | 5 |
| right_1n_mirror | drop10_g100 | FALL 2.505s | -0.015 | 32768 | 5 |
| handle_forward_4n | drop5_g000 | FALL 3.385s | -1.245 | 0 | 0 |
| handle_forward_4n | drop5_g025 | RECOVERED | — | 8192 | 23 |
| handle_forward_4n | drop5_g045 | FALL 4.315s | -0.315 | 14746 | 17 |
| handle_forward_4n | drop5_g050 | FALL 4.055s | -0.575 | 16384 | 16 |
| handle_forward_4n | drop5_g075 | FALL 2.850s | -1.780 | 24576 | 11 |
| handle_forward_4n | drop5_g095 | FALL 4.880s | +0.250 | 31130 | 19 |
| handle_forward_4n | drop5_g100 | FALL 5.200s | +0.570 | 32768 | 20 |
| handle_forward_4n | drop10_g000 | UNSETTLED | — | 0 | 0 |
| handle_forward_4n | drop10_g025 | FALL 3.215s | -1.415 | 8192 | 6 |
| handle_forward_4n | drop10_g045 | FALL 5.310s | +0.680 | 14746 | 10 |
| handle_forward_4n | drop10_g050 | FALL 5.445s | +0.815 | 16384 | 10 |
| handle_forward_4n | drop10_g075 | FALL 2.915s | -1.715 | 24576 | 5 |
| handle_forward_4n | drop10_g095 | UNSETTLED | — | 31130 | 11 |
| handle_forward_4n | drop10_g100 | FALL 2.975s | -1.655 | 32768 | 5 |
| forward_4n_friction_0p03 | drop5_g000 | FALL 1.700s | +0.090 | 0 | 0 |
| forward_4n_friction_0p03 | drop5_g025 | FALL 1.635s | +0.025 | 8192 | 6 |
| forward_4n_friction_0p03 | drop5_g045 | FALL 1.655s | +0.045 | 14746 | 6 |
| forward_4n_friction_0p03 | drop5_g050 | FALL 1.720s | +0.110 | 16384 | 6 |
| forward_4n_friction_0p03 | drop5_g075 | FALL 1.715s | +0.105 | 24576 | 6 |
| forward_4n_friction_0p03 | drop5_g095 | FALL 1.625s | +0.015 | 31130 | 6 |
| forward_4n_friction_0p03 | drop5_g100 | FALL 1.635s | +0.025 | 32768 | 6 |
| forward_4n_friction_0p03 | drop10_g000 | FALL 1.595s | -0.015 | 0 | 0 |
| forward_4n_friction_0p03 | drop10_g025 | FALL 1.695s | +0.085 | 8192 | 3 |
| forward_4n_friction_0p03 | drop10_g045 | FALL 1.690s | +0.080 | 14746 | 3 |
| forward_4n_friction_0p03 | drop10_g050 | FALL 1.605s | -0.005 | 16384 | 3 |
| forward_4n_friction_0p03 | drop10_g075 | FALL 1.650s | +0.040 | 24576 | 3 |
| forward_4n_friction_0p03 | drop10_g095 | FALL 1.695s | +0.085 | 31130 | 3 |
| forward_4n_friction_0p03 | drop10_g100 | FALL 1.630s | +0.020 | 32768 | 3 |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| all_arms_replay_exact | PASS |
| exact_stream_is_unchanged_by_dormant_gain | PASS |
| typed_hold_is_unavailable_only_and_q15_exact | PASS |
| gain_zero_is_withheld_semantics | PASS |
| finite_without_numeric_fault | PASS |
| zero_rust_allocation_and_python_gc | PASS |

## Interpretation

- Gain is quantized once to Q15 by Rust and applied to the cached admitted effort without compounding. Exact-observation behavior is dormant and must remain bit-identical for zero, partial, and full configured gain.
- Zero gain disables retained authority and is required to be semantically identical to the corresponding withheld arm; executable retained-zero provenance is forbidden.
- Duration-specific admitted gains: **{'drop5': [], 'drop10': [0.45, 0.95]}**. Gains admitted across both loss durations: **none**.
- The burst duration is not known on the first unavailable tick. A gain that passes one declared fault class is not a causal global selector; the next candidate must use only current state/history or a separately authenticated transport guarantee.
