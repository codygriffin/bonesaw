# Bonesaw conditional primary/contingency authority A/B · r177

> Mechanism **PASS** · controller promotion **REJECTED**. Primary r137 contact authority remains unchanged until the separately admitted R175 action owns the command.

## Consequence

| case | r137 | measured negative | conditional candidate | selected ticks | r137 fall Δ s |
|---|---|---|---|---|---|
| backward_2n | RECOVERED | FALL 3.420s | RECOVERED | 2 | — |
| backward_4n | RECOVERED | FALL 2.730s | RECOVERED | 3 | — |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.450s | 88 | +0.325 |
| forward_2n | RECOVERED | FALL 3.680s | RECOVERED | 2 | — |
| forward_4n_reference | RECOVERED | FALL 2.865s | RECOVERED | 5 | — |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 1.525s | 55 | -3.295 |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 1.960s | 50 | -0.510 |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 1.925s | 61 | -0.425 |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 1.555s | 49 | -1.055 |
| nominal | RECOVERED | FALL 3.395s | RECOVERED | 0 | — |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.550s | 92 | +0.535 |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.165s | 49 | -0.250 |
| up_4n | RECOVERED | FALL 4.215s | RECOVERED | 0 | — |
| down_4n | RECOVERED | FALL 3.450s | RECOVERED | 0 | — |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 2.280s | 114 | -2.350 |
| short_8n_50ms | RECOVERED | FALL 1.785s | RECOVERED | 6 | — |
| long_2n_200ms | RECOVERED | FALL 3.155s | RECOVERED | 3 | — |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | RECOVERED | 3 | — |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 7 | — |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 1.525s | 53 | -0.085 |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| shadow_preserves_r137_command_and_plant_exactly | PASS |
| candidate_replay_exact | PASS |
| selection_exercised | PASS |
| selection_only_after_admission | PASS |
| selection_only_after_exact_support_loss | PASS |
| candidate_finite | PASS |
| zero_timed_rust_allocation | PASS |
| zero_python_gc | PASS |

## Promotion gates

| gate | result |
|---|---|
| all_r137_green_rows_preserved | PASS |
| no_r137_fall_boundary_earlier | FAIL |
| no_numeric_fault | PASS |
| zero_5ms_loop_overruns | FAIL |

## Authority contract

- Exact measured support is consumed by the non-executing R175 author/WBC. The r137 primary WBC retains its established hard-support program until the admitted contingency is selected; a rejected candidate falls back to ordinary r137 freshness rather than a measured-contact primary command.
- The shadow must preserve r137 torque and plant state bit-for-bit. Selection still requires exact double-support arming, later exact support loss, typed candidate admission, and hard violation below `1e-8`.
- No blend, elapsed-time dwell, command-lifetime extension, cache resurrection, rejected-row execution, or reset participates.
