# Bonesaw observation-delay startup A/B · r183

> Causal classification **PASS** · synchronous profile **REJECTED**. This audit distinguishes a steady delayed stream from introducing delay discontinuously when actuator authority begins.

## Result

| case | profile | exact | candidate | fall Δ s | timestamp faults | withheld | plant/command exact |
|---|---|---|---|---|---|---|---|
| nominal | delay_onset_20ms | RECOVERED | RECOVERED | — | 4 | 5 | NO |
| nominal | steady_delay_20ms | RECOVERED | RECOVERED | — | 0 | 0 | YES |
| forward_4n_reference | delay_onset_20ms | RECOVERED | RECOVERED | — | 4 | 5 | NO |
| forward_4n_reference | steady_delay_20ms | RECOVERED | RECOVERED | — | 0 | 0 | YES |
| backward_4n | delay_onset_20ms | RECOVERED | RECOVERED | — | 4 | 5 | NO |
| backward_4n | steady_delay_20ms | RECOVERED | RECOVERED | — | 0 | 0 | YES |
| left_1n | delay_onset_20ms | FALL 2.470s | FALL 2.760s | +0.290 | 4 | 5 | NO |
| left_1n | steady_delay_20ms | FALL 2.470s | FALL 2.470s | +0.000 | 0 | 0 | YES |
| right_1n_mirror | delay_onset_20ms | FALL 2.520s | FALL 2.530s | +0.010 | 4 | 5 | NO |
| right_1n_mirror | steady_delay_20ms | FALL 2.520s | FALL 2.520s | +0.000 | 0 | 0 | YES |
| handle_forward_4n | delay_onset_20ms | FALL 4.630s | FALL 3.005s | -1.625 | 4 | 5 | NO |
| handle_forward_4n | steady_delay_20ms | FALL 4.630s | FALL 4.630s | +0.000 | 0 | 0 | YES |
| forward_4n_friction_0p03 | delay_onset_20ms | FALL 1.610s | FALL 0.625s | -0.985 | 4 | 4 | NO |
| forward_4n_friction_0p03 | steady_delay_20ms | FALL 1.610s | FALL 1.610s | +0.000 | 0 | 0 | YES |

## Causal gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| all_arms_replay_exact | PASS |
| delay_onset_exercises_timestamp_fault | PASS |
| delay_onset_timestamp_faults_fail_closed | PASS |
| steady_delay_is_20ms_from_first_tick | PASS |
| steady_delay_never_withholds | PASS |
| steady_delay_plant_and_command_are_bit_exact | PASS |
| steady_delay_terminal_outcome_is_exact | PASS |
| finite_without_numeric_fault | PASS |
| zero_rust_allocation_and_python_gc | PASS |

## Timing

| metric | value |
|---|---|
| loop misses >5 ms | 17 |
| worst loop | 6.050 ms |
| worst controller call | 5.569 ms |

## Interpretation

- The r182 20 ms arm is a delay-onset fault: zero-age prestart samples are followed by older runtime samples. Rust correctly rejects and withholds on four backwards timestamps; most rows need one additional activation tick before authority returns.
- The steady-delay arm advances the receiver clock, then primes three already-aged samples before actuator authority. Every runtime sample is exactly 20 ms old and monotonically newer than the previous acquisition.
- Exact plant/command equality under steady delay is specific to r181 fixed-effective-effort realization: observation age changes the proof route, but it cannot change the applied torque. It is not a generic claim that delayed contact is harmless.
- Periodic unavailability remains a real open consequence problem; this audit does not reclassify the r182 dropout regressions.
