# Bonesaw inexact-observation hold A/B · r185

> Mechanism **PASS** · consequence **REJECTED** · synchronous profile **REJECTED**. Rust may replay only the last already-admitted effective effort for a separately configured number of non-exact observation ticks.

## Result

| case | profile | exact | candidate | fall Δ s | unavailable | withheld | typed holds |
|---|---|---|---|---|---|---|---|
| nominal | drop5_hold0 | RECOVERED | RECOVERED | — | 23 | 23 | 0 |
| nominal | drop5_hold1 | RECOVERED | RECOVERED | — | 23 | 0 | 23 |
| nominal | drop10_hold1 | RECOVERED | RECOVERED | — | 22 | 11 | 11 |
| nominal | drop10_hold2 | RECOVERED | RECOVERED | — | 22 | 0 | 22 |
| nominal | drop10_hold0 | RECOVERED | RECOVERED | — | 22 | 22 | 0 |
| forward_4n_reference | drop5_hold0 | RECOVERED | RECOVERED | — | 23 | 23 | 0 |
| forward_4n_reference | drop5_hold1 | RECOVERED | RECOVERED | — | 23 | 0 | 23 |
| forward_4n_reference | drop10_hold1 | RECOVERED | RECOVERED | — | 22 | 11 | 11 |
| forward_4n_reference | drop10_hold2 | RECOVERED | RECOVERED | — | 22 | 0 | 22 |
| forward_4n_reference | drop10_hold0 | RECOVERED | RECOVERED | — | 22 | 22 | 0 |
| backward_4n | drop5_hold0 | RECOVERED | RECOVERED | — | 23 | 23 | 0 |
| backward_4n | drop5_hold1 | RECOVERED | RECOVERED | — | 23 | 0 | 23 |
| backward_4n | drop10_hold1 | RECOVERED | RECOVERED | — | 22 | 11 | 11 |
| backward_4n | drop10_hold2 | RECOVERED | RECOVERED | — | 22 | 0 | 22 |
| backward_4n | drop10_hold0 | RECOVERED | RECOVERED | — | 22 | 22 | 0 |
| left_1n | drop5_hold0 | FALL 2.470s | FALL 1.785s | -0.685 | 7 | 7 | 0 |
| left_1n | drop5_hold1 | FALL 2.470s | FALL 2.430s | -0.040 | 9 | 0 | 9 |
| left_1n | drop10_hold1 | FALL 2.470s | FALL 4.175s | +1.705 | 16 | 14 | 8 |
| left_1n | drop10_hold2 | FALL 2.470s | FALL 1.780s | -0.690 | 6 | 0 | 6 |
| left_1n | drop10_hold0 | FALL 2.470s | FALL 2.725s | +0.255 | 10 | 12 | 0 |
| right_1n_mirror | drop5_hold0 | FALL 2.520s | FALL 2.170s | -0.350 | 8 | 8 | 0 |
| right_1n_mirror | drop5_hold1 | FALL 2.520s | FALL 3.035s | +0.515 | 12 | 0 | 12 |
| right_1n_mirror | drop10_hold1 | FALL 2.520s | FALL 2.505s | -0.015 | 9 | 4 | 5 |
| right_1n_mirror | drop10_hold2 | FALL 2.520s | FALL 3.175s | +0.655 | 12 | 0 | 12 |
| right_1n_mirror | drop10_hold0 | FALL 2.520s | FALL 2.365s | -0.155 | 8 | 9 | 0 |
| handle_forward_4n | drop5_hold0 | FALL 4.630s | FALL 3.385s | -1.245 | 13 | 14 | 0 |
| handle_forward_4n | drop5_hold1 | FALL 4.630s | FALL 5.200s | +0.570 | 20 | 0 | 20 |
| handle_forward_4n | drop10_hold1 | FALL 4.630s | FALL 2.975s | -1.655 | 10 | 5 | 5 |
| handle_forward_4n | drop10_hold2 | FALL 4.630s | FALL 3.690s | -0.940 | 14 | 0 | 14 |
| handle_forward_4n | drop10_hold0 | FALL 4.630s | UNSETTLED | — | 22 | 22 | 0 |
| forward_4n_friction_0p03 | drop5_hold0 | FALL 1.610s | FALL 1.700s | +0.090 | 6 | 6 | 0 |
| forward_4n_friction_0p03 | drop5_hold1 | FALL 1.610s | FALL 1.635s | +0.025 | 6 | 0 | 6 |
| forward_4n_friction_0p03 | drop10_hold1 | FALL 1.610s | FALL 1.630s | +0.020 | 6 | 3 | 3 |
| forward_4n_friction_0p03 | drop10_hold2 | FALL 1.610s | FALL 1.585s | -0.025 | 6 | 0 | 6 |
| forward_4n_friction_0p03 | drop10_hold0 | FALL 1.610s | FALL 1.595s | -0.015 | 6 | 6 | 0 |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| all_arms_replay_exact | PASS |
| exact_stream_is_unchanged_by_dormant_hold_config | PASS |
| typed_hold_is_only_unavailable_and_replays_preceding_effort | PASS |
| typed_hold_count_matches_burst_budget | PASS |
| finite_without_numeric_fault | PASS |
| zero_rust_allocation_and_python_gc | PASS |
| no_exact_green_case_becomes_a_fall | PASS |
| no_existing_fall_boundary_moves_earlier | FAIL |
| zero_5ms_loop_overruns | FAIL |
| all_controller_calls_below_5ms | FAIL |

## Contract

- The hold is a distinct Rust authority selection and provenance. It cannot execute at startup, on an exact contact transition, after its tick budget, or without a prior admitted command.
- Held ticks replay the preceding effective actuator effort exactly, emit no contact-force witness, and do not refresh primary-command health.
- Hold configuration is dormant under exact observation; exact hold-0/1/2 semantic traces must be identical.
- Earlier candidate boundaries: **6** (left_1n/drop5_hold1, left_1n/drop10_hold2, right_1n_mirror/drop10_hold1, handle_forward_4n/drop10_hold1, handle_forward_4n/drop10_hold2, forward_4n_friction_0p03/drop10_hold2). New green falls: **0** (none).
