# Bonesaw observation-dropout phase A/B · r184

> Mechanism **PASS** · consequence **REJECTED** · synchronous profile **REJECTED**. This audit separates dropout on actuator tick zero from the same periodic loss after authority is established.

## Result

| case | profile | exact | candidate | fall Δ s | unavailable | withheld | terminal exact |
|---|---|---|---|---|---|---|---|
| nominal | startup_5ms_per_250ms | RECOVERED | RECOVERED | — | 24 | 24 | NO |
| nominal | established_5ms_per_250ms | RECOVERED | RECOVERED | — | 23 | 23 | NO |
| nominal | startup_10ms_per_500ms | RECOVERED | RECOVERED | — | 24 | 24 | NO |
| nominal | established_10ms_per_500ms | RECOVERED | RECOVERED | — | 22 | 22 | NO |
| forward_4n_reference | startup_5ms_per_250ms | RECOVERED | RECOVERED | — | 24 | 24 | NO |
| forward_4n_reference | established_5ms_per_250ms | RECOVERED | RECOVERED | — | 23 | 23 | NO |
| forward_4n_reference | startup_10ms_per_500ms | RECOVERED | RECOVERED | — | 24 | 24 | NO |
| forward_4n_reference | established_10ms_per_500ms | RECOVERED | RECOVERED | — | 22 | 22 | NO |
| backward_4n | startup_5ms_per_250ms | RECOVERED | RECOVERED | — | 24 | 24 | NO |
| backward_4n | established_5ms_per_250ms | RECOVERED | RECOVERED | — | 23 | 23 | NO |
| backward_4n | startup_10ms_per_500ms | RECOVERED | RECOVERED | — | 24 | 24 | NO |
| backward_4n | established_10ms_per_500ms | RECOVERED | RECOVERED | — | 22 | 22 | NO |
| left_1n | startup_5ms_per_250ms | FALL 2.470s | FALL 1.920s | -0.550 | 8 | 10 | NO |
| left_1n | established_5ms_per_250ms | FALL 2.470s | FALL 1.785s | -0.685 | 7 | 7 | NO |
| left_1n | startup_10ms_per_500ms | FALL 2.470s | FALL 3.340s | +0.870 | 14 | 15 | NO |
| left_1n | established_10ms_per_500ms | FALL 2.470s | FALL 2.725s | +0.255 | 10 | 12 | NO |
| right_1n_mirror | startup_5ms_per_250ms | FALL 2.520s | FALL 2.275s | -0.245 | 10 | 10 | NO |
| right_1n_mirror | established_5ms_per_250ms | FALL 2.520s | FALL 2.170s | -0.350 | 8 | 8 | NO |
| right_1n_mirror | startup_10ms_per_500ms | FALL 2.520s | FALL 2.975s | +0.455 | 12 | 13 | NO |
| right_1n_mirror | established_10ms_per_500ms | FALL 2.520s | FALL 2.365s | -0.155 | 8 | 9 | NO |
| handle_forward_4n | startup_5ms_per_250ms | FALL 4.630s | FALL 4.155s | -0.475 | 17 | 18 | NO |
| handle_forward_4n | established_5ms_per_250ms | FALL 4.630s | FALL 3.385s | -1.245 | 13 | 14 | NO |
| handle_forward_4n | startup_10ms_per_500ms | FALL 4.630s | FALL 4.305s | -0.325 | 18 | 18 | NO |
| handle_forward_4n | established_10ms_per_500ms | FALL 4.630s | UNSETTLED | — | 22 | 22 | NO |
| forward_4n_friction_0p03 | startup_5ms_per_250ms | FALL 1.610s | FALL 1.635s | +0.025 | 7 | 7 | NO |
| forward_4n_friction_0p03 | established_5ms_per_250ms | FALL 1.610s | FALL 1.700s | +0.090 | 6 | 6 | NO |
| forward_4n_friction_0p03 | startup_10ms_per_500ms | FALL 1.610s | FALL 0.690s | -0.920 | 4 | 4 | NO |
| forward_4n_friction_0p03 | established_10ms_per_500ms | FALL 1.610s | FALL 1.595s | -0.015 | 6 | 6 | NO |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| all_arms_replay_exact | PASS |
| all_dropout_profiles_exercised | PASS |
| every_unavailable_tick_fails_closed | PASS |
| finite_without_numeric_fault | PASS |
| zero_rust_allocation_and_python_gc | PASS |

## Consequence and timing gates

| gate | result |
|---|---|
| no_exact_green_case_becomes_a_fall | PASS |
| no_existing_fall_boundary_moves_earlier | FAIL |
| zero_5ms_loop_overruns | FAIL |
| all_controller_calls_below_5ms | FAIL |

## Interpretation

- Startup profiles lose the first observation and repeat at the declared period. Established profiles defer their first loss by one full period, after primary authority has executed continuously.
- Both profiles retain a zero-tick command lease. Unavailable evidence must withhold; this audit classifies phase sensitivity before introducing a replacement command.
- Earlier existing-fall boundaries: **10** (left_1n/startup_5ms_per_250ms, left_1n/established_5ms_per_250ms, right_1n_mirror/startup_5ms_per_250ms, right_1n_mirror/established_5ms_per_250ms, right_1n_mirror/established_10ms_per_500ms, handle_forward_4n/startup_5ms_per_250ms, handle_forward_4n/established_5ms_per_250ms, handle_forward_4n/startup_10ms_per_500ms, forward_4n_friction_0p03/startup_10ms_per_500ms, forward_4n_friction_0p03/established_10ms_per_500ms). New exact-green falls: **0** (none).
