# Bonesaw deterministic cross-tick planner cadence A/B · r171

> Timing profile **PASS** · physical profile **FAIL** · controller promotion **NO**.

## Result

The candidate runs the speculative planner once every **3 control ticks**. Rust request supervision treats intervening ticks as intentional no-update holds, while confirmation expires only beyond the same explicit period. Scheduling depends only on tick sequence, never measured wall time.
Across **20** frozen cases, synchronous misses change **3 → 0** and planner WBC queries change **20,854 → 6,904**. The worst physical boundary moves **-0.095 s** and executable-request ticks change **26 → 0**.

| case | period 1 | period 3 | boundary Δs | miss P1 | miss Px | queries P1 | queries Px | request P1 | request Px | replay |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | FALL 3.395s | -0.010 | 0 | 0 | 795 | 263 | 3 | 0 | YES |
| forward_2n | FALL 3.680s | FALL 3.680s | +0.000 | 1 | 0 | 1375 | 453 | 4 | 0 | YES |
| forward_4n_reference | FALL 2.865s | FALL 2.865s | +0.000 | 1 | 0 | 1383 | 452 | 0 | 0 | YES |
| forward_6n_overload | FALL 1.230s | FALL 1.230s | +0.000 | 0 | 0 | 345 | 115 | 1 | 0 | YES |
| backward_2n | FALL 3.420s | FALL 3.420s | +0.000 | 0 | 0 | 1308 | 435 | 0 | 0 | YES |
| backward_4n | FALL 2.730s | FALL 2.730s | +0.000 | 0 | 0 | 1395 | 464 | 0 | 0 | YES |
| left_1n | FALL 2.295s | FALL 2.295s | +0.000 | 0 | 0 | 741 | 246 | 0 | 0 | YES |
| left_2n | FALL 2.630s | FALL 2.535s | -0.095 | 0 | 0 | 748 | 214 | 18 | 0 | YES |
| left_4n | FALL 1.850s | FALL 1.850s | +0.000 | 0 | 0 | 658 | 214 | 0 | 0 | YES |
| right_2n | FALL 2.030s | FALL 2.030s | +0.000 | 0 | 0 | 520 | 175 | 0 | 0 | YES |
| right_4n | FALL 2.195s | FALL 2.195s | +0.000 | 0 | 0 | 865 | 294 | 0 | 0 | YES |
| diagonal_4n | FALL 2.605s | FALL 2.605s | +0.000 | 0 | 0 | 1235 | 408 | 0 | 0 | YES |
| up_4n | FALL 4.215s | FALL 4.215s | +0.000 | 0 | 0 | 1383 | 476 | 0 | 0 | YES |
| down_4n | FALL 3.450s | FALL 3.450s | +0.000 | 0 | 0 | 942 | 314 | 0 | 0 | YES |
| handle_forward_4n | FALL 2.045s | FALL 2.045s | +0.000 | 0 | 0 | 910 | 299 | 0 | 0 | YES |
| short_8n_50ms | FALL 1.785s | FALL 1.785s | +0.000 | 0 | 0 | 717 | 236 | 0 | 0 | YES |
| long_2n_200ms | FALL 3.155s | FALL 3.155s | +0.000 | 0 | 0 | 1588 | 526 | 0 | 0 | YES |
| forward_2n_three_pulses | FALL 3.625s | FALL 3.625s | +0.000 | 1 | 0 | 1898 | 629 | 0 | 0 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1836 | 616 | 0 | 0 | YES |
| forward_4n_friction_0p03 | FALL 0.715s | FALL 0.715s | +0.000 | 0 | 0 | 212 | 75 | 0 | 0 | YES |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| control_cadence_exact | PASS |
| candidate_cadence_exact | PASS |
| candidate_replay_exact | PASS |
| zero_synchronous_deadline_misses | PASS |
| zero_final_wbc_deadline_misses | PASS |
| no_earlier_physical_boundary | FAIL |
| no_new_numeric_fault | PASS |
| request_only_after_confirmation | PASS |
| request_only_after_hybrid_guard | PASS |
| candidate_age_bounded_by_period | PASS |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |

## Interpretation

- The mechanism is a bounded temporal contract, not an asynchronous thread and not a wall-clock heuristic.
- A skipped tick cannot be reported as a failed planner update; candidate age, confirmation gap, and request freshness remain explicit Rust state.
- Timing admission and physical non-regression are separate. A zero-miss cadence is rejected if it suppresses useful requests or moves any retained boundary earlier.
- No result promotes the experimental viability action, r169 cache, or controller. R137 remains live.

## Scope

Host timing is descriptive for `-C target-cpu=native`. The evaluation uses the retained Upkie MuJoCo plant and has no hardware, estimator delay/noise, real-time kernel, or authenticated transport claim.
