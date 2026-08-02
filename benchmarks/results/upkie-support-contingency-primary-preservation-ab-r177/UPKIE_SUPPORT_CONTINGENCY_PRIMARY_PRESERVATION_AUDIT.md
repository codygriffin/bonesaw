# Bonesaw primary-support preservation diagnostic · r177

> Mechanism **PASS** · controller promotion **REJECTED**. This diagnostic isolates the primary-support discontinuity found by r176; preservation is an experiment, not authority to ignore observed support.

## Consequence

| case | r137 | measured | preserved shadow | selected | selected ticks | fall Δ vs r137 s |
|---|---|---|---|---|---|---|
| backward_2n | RECOVERED | FALL 3.420s | RECOVERED | RECOVERED | 2 | — |
| backward_4n | RECOVERED | FALL 2.730s | RECOVERED | RECOVERED | 3 | — |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.125s | FALL 2.450s | 88 | +0.325 |
| down_4n | RECOVERED | FALL 3.450s | RECOVERED | RECOVERED | 0 | — |
| forward_2n | RECOVERED | FALL 3.680s | RECOVERED | RECOVERED | 2 | — |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | RECOVERED | RECOVERED | 3 | — |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 1.610s | FALL 1.525s | 53 | -0.085 |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | RECOVERED | 7 | — |
| forward_4n_reference | RECOVERED | FALL 2.865s | RECOVERED | RECOVERED | 5 | — |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 4.820s | FALL 1.525s | 55 | -3.295 |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 4.630s | FALL 2.280s | 114 | -2.350 |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 2.470s | FALL 1.960s | 50 | -0.510 |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 2.350s | FALL 1.925s | 61 | -0.425 |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 2.610s | FALL 1.555s | 49 | -1.055 |
| long_2n_200ms | RECOVERED | FALL 3.155s | RECOVERED | RECOVERED | 3 | — |
| nominal | RECOVERED | FALL 3.395s | RECOVERED | RECOVERED | 0 | — |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.015s | FALL 2.550s | 92 | +0.535 |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.415s | FALL 2.165s | 49 | -0.250 |
| short_8n_50ms | RECOVERED | FALL 1.785s | RECOVERED | RECOVERED | 6 | — |
| up_4n | RECOVERED | FALL 4.215s | RECOVERED | RECOVERED | 0 | — |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| preserved_primary_shadow_matches_r137_physics | PASS |
| candidate_replay_exact | PASS |
| selection_exercised | PASS |
| selection_only_after_admission | PASS |
| selection_only_after_causal_arming_request | PASS |
| selection_only_after_exact_support_loss | PASS |
| selected_rows_are_typed_executable | PASS |
| candidate_finite | PASS |
| zero_timed_rust_allocation | PASS |
| zero_python_gc | PASS |

## Promotion gates

| gate | result |
|---|---|
| primary_support_has_current_observation_authority | FAIL |
| retained_green_rows_preserved | PASS |
| no_r137_fall_boundary_earlier | FAIL |
| no_candidate_numeric_fault | PASS |
| zero_5ms_loop_overruns | FAIL |

## Interpretation

- The r139 filter intentionally begins with no hard-contact authority and needs three positive samples to activate contact. R176 let the primary WBC consume that startup/reacquisition state, which changed torque before the contingency selector was armed.
- This arm keeps the established double-support primary solve physically identical to r137 while the exact observed mask remains available only to the independent contingency query and selector. Exact equality is checked on plant state, torque, status, contact count, fall-safe state, and terminal consequence; observation telemetry is deliberately excluded.
- The zero-overrun gate is not a candidate-only regression: retained r137, preserved shadow, and selected candidate record 642, 645, and 202 misses respectively. Candidate termination shortens several red traces, so these totals are not normalized speed ratios.
- Preserving a fictitious primary support row is not itself promotable. If it restores the envelope, the follow-up must replace it with an explicit enable/reacquisition state and bounded retained-command authority, then repeat the same plant gates with delay/noise/dropout.
