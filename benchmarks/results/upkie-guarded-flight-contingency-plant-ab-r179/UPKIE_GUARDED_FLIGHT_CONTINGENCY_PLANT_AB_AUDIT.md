# Bonesaw guarded flight-contingency authority A/B · r179

> Mechanism **PASS** · conditioned action in the preserved-primary scaffold **PASS** · plant-controller promotion **REJECTED**. One exact flight transition may transfer authority only when the Rust eight-knot forecast improves by at least `0.10` inside a `40 rad/s²` angular-acceleration trust region.

## Consequence

| case | r137 | measured negative | guarded candidate | selected ticks | forecast Δ | r137 fall Δ s |
|---|---|---|---|---|---|---|
| backward_2n | RECOVERED | FALL 3.420s | RECOVERED | 1 | +0.507 | — |
| backward_4n | RECOVERED | FALL 2.730s | RECOVERED | 1 | +1.221 | — |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.325s | 1 | +0.121 | +0.200 |
| down_4n | RECOVERED | FALL 3.450s | RECOVERED | 0 | — | — |
| forward_2n | RECOVERED | FALL 3.680s | RECOVERED | 1 | +0.623 | — |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | RECOVERED | 1 | +0.623 | — |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 1.665s | 1 | +0.939 | +0.055 |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 1 | +0.733 | — |
| forward_4n_reference | RECOVERED | FALL 2.865s | RECOVERED | 1 | +0.733 | — |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 4.820s | 0 | +1.750 | +0.000 |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 4.665s | 1 | +0.642 | +0.035 |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 2.470s | 0 | -0.606 | +0.000 |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 2.350s | 0 | -0.995 | +0.000 |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 2.610s | 0 | +0.056 | +0.000 |
| long_2n_200ms | RECOVERED | FALL 3.155s | RECOVERED | 1 | +0.691 | — |
| nominal | RECOVERED | FALL 3.395s | RECOVERED | 0 | — | — |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.015s | 0 | -0.085 | +0.000 |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.415s | 0 | +0.089 | +0.000 |
| short_8n_50ms | RECOVERED | FALL 1.785s | RECOVERED | 1 | +1.270 | — |
| up_4n | RECOVERED | FALL 4.215s | RECOVERED | 0 | — | — |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| shadow_preserves_r137_command_and_plant_exactly | PASS |
| candidate_replay_exact | PASS |
| selection_exercised | PASS |
| selection_only_after_admission | PASS |
| selection_only_after_exact_support_loss | PASS |
| single_evaluation_lease_observed | PASS |
| selection_only_in_exact_flight | PASS |
| selection_only_after_forecast_guard | PASS |
| guard_acceptance_and_rejection_exercised | PASS |
| selected_forecast_margin_at_least_0p10 | PASS |
| selected_angular_acceleration_within_40_rad_s2 | PASS |
| candidate_finite | PASS |
| zero_timed_rust_allocation | PASS |
| zero_python_gc | PASS |

## Conditioned-action gates

| gate | result |
|---|---|
| all_r137_green_rows_preserved | PASS |
| no_r137_fall_boundary_earlier | PASS |
| no_numeric_fault | PASS |
| no_incremental_total_loop_overruns | PASS |
| green_rows_keep_5ms_loop_budget | PASS |
| at_least_one_fall_boundary_improved | PASS |

## Plant-controller gates

| gate | result |
|---|---|
| conditioned_action_passes_preserved_primary_scaffold | PASS |
| generic_current_observation_authority_integrated | FAIL |

## Conditional timing

| work | min µs | p50 µs | p95 µs | max µs |
|---|---|---|---|---|
| candidate WBC | 121.720 | 127.461 | 137.001 | 143.902 |
| Rust forecast guard | 0.371 | 0.381 | 0.521 | 0.521 |

Maximum complete loop on any retained green row: `1.113 ms`. Whole red-case overrun counts are `642→592`; these include post-instability solver tails and are not substituted for the conditional query measurements.

## Authority contract

- The primary retains r137 double-support authority. After exact double-support arming, only the first exact mask-0 flight transition receives a candidate query; the evaluation lease is consumed whether the guard accepts or rejects it.
- The R175 flight author requests exact ballistic gravity and no fictitious horizontal support force. Selection also requires typed WBC admission, hard violation below `1e-8`, at least `0.10` Rust forecast improvement over inertial continuation, and achieved root angular acceleration no greater than `40 rad/s²`.
- Rejection executes the ordinary r137 fresh command. There is no blend, unilateral-support transfer, repeated opportunistic query, elapsed-time dwell, cache resurrection, or rejected-row execution.
- This deliberately remains a preserved-primary scaffold. R178 contact-program authority is not enabled in this A/B, so the primary still lacks current observation authority during true flight; controller promotion remains rejected until the guarded action and bounded reacquisition are composed without regressing these results.
