# Bonesaw current-support realization ablation · r181

> Mechanism **PASS** · semantic composition **ADMITTED** · synchronous profile **REJECTED** · hardware controller **NOT PROMOTED**. This ablation isolates why the generic R178 authority must realize the retained r137 torque instead of directly selecting an independently optimized current-support action.

## Result

| case | r137 | raw transfer | realized transfer | primary ticks | current ticks | fallbacks | plant exact |
|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | RECOVERED | 1200 | 0 | 0 | YES |
| forward_2n | RECOVERED | RECOVERED | RECOVERED | 1199 | 1 | 0 | YES |
| forward_4n_reference | RECOVERED | RECOVERED | RECOVERED | 1197 | 3 | 0 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.395s | FALL 4.820s | 849 | 115 | 0 | YES |
| backward_2n | RECOVERED | RECOVERED | RECOVERED | 1199 | 1 | 0 | YES |
| backward_4n | RECOVERED | RECOVERED | RECOVERED | 1196 | 4 | 0 | YES |
| left_1n | FALL 2.470s | FALL 2.390s | FALL 2.470s | 356 | 138 | 0 | YES |
| left_2n | FALL 2.350s | FALL 2.895s | FALL 2.350s | 331 | 139 | 0 | YES |
| left_4n | FALL 2.610s | FALL 2.170s | FALL 2.610s | 223 | 299 | 0 | YES |
| right_2n | FALL 2.015s | FALL 1.835s | FALL 2.015s | 375 | 28 | 0 | YES |
| right_4n | FALL 2.415s | FALL 2.190s | FALL 2.415s | 387 | 96 | 0 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.385s | FALL 2.125s | 313 | 112 | 0 | YES |
| up_4n | RECOVERED | RECOVERED | RECOVERED | 1200 | 0 | 0 | YES |
| down_4n | RECOVERED | RECOVERED | RECOVERED | 1200 | 0 | 0 | YES |
| handle_forward_4n | FALL 4.630s | FALL 2.365s | FALL 4.630s | 725 | 201 | 0 | YES |
| short_8n_50ms | RECOVERED | RECOVERED | RECOVERED | 1194 | 6 | 0 | YES |
| long_2n_200ms | RECOVERED | RECOVERED | RECOVERED | 1195 | 5 | 0 | YES |
| forward_2n_three_pulses | RECOVERED | RECOVERED | RECOVERED | 1198 | 2 | 0 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 1194 | 6 | 0 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 1.560s | FALL 1.610s | 251 | 71 | 0 | YES |

The raw-current-support negative control changes the terminal boundary or qualification of `16` row(s): `forward_2n, forward_4n_reference, forward_6n_overload, backward_2n, backward_4n, left_1n, left_2n, left_4n, right_2n, right_4n, diagonal_4n, handle_forward_4n, short_8n_50ms, long_2n_200ms, forward_2n_three_pulses, forward_4n_friction_0p03`. The realized composition changes none. Across `18209` control ticks, the authority selected the retained primary proof `16982` times and its current-support realization `1227` times, with `0` withheld and `0` lease ticks.

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| raw_transfer_negative_control_changes_outcomes | PASS |
| current_support_path_exercised | PASS |
| all_control_ticks_have_executable_program | PASS |
| hard_mask_is_exact_raw_stable_intersection | PASS |
| contact_program_masks_are_consistent | PASS |
| current_support_selection_is_typed_admitted | PASS |
| current_support_hard_violation_below_1e_8 | PASS |
| no_realization_fallback | PASS |
| executed_torque_is_bit_exact_primary_program | PASS |
| candidate_replay_exact | PASS |
| candidate_finite | PASS |
| zero_timed_rust_allocation | PASS |
| zero_python_gc | PASS |

## Non-regression gates

| gate | result |
|---|---|
| r137_plant_and_command_trace_bit_exact | PASS |
| r137_terminal_outcome_exact | PASS |
| all_r137_green_rows_preserved | PASS |
| no_r137_fall_boundary_earlier | PASS |
| all_r137_fall_boundaries_exact | PASS |
| no_numeric_fault | PASS |
| green_rows_keep_5ms_loop_budget | PASS |

## Scheduling and deployment gates

| gate | result |
|---|---|
| zero_5ms_loop_overruns | FAIL |
| all_controller_calls_below_5ms | FAIL |
| semantic_current_observation_composition_admitted | PASS |
| synchronous_ordinary_process_profile_admitted | FAIL |
| delay_noise_dropout_and_mirrored_evidence_complete | FAIL |
| higher_fidelity_or_hardware_evidence_complete | FAIL |

## Timing and memory

Worst per-case current-support WBC p99: `108.011 µs`; maximum single solve: `128.713 µs`; maximum complete green-row loop: `0.692 ms`. Timed Rust allocations and Python GC collections are zero. Whole-run overrun counts are `641→10` and include red-case post-instability tails.

## Authority stack

1. Three exact pre-control contact samples initialize the debouncer before torque authority is enabled; startup therefore has no zero-command hole.
2. The r137 primary program supplies either a fresh raw torque or its already-bounded freshness-faded command. Its raw freshness alone drives fall-safe confidence.
3. A fixed-effort Rust WBC receives that exact torque and the current hard contact mask. It may prove achieved acceleration and feasibility, but it cannot optimize or alter the torque.
4. R178 selects the primary proof when its authored mask remains current and the current-support proof otherwise. A failed realization is non-executable; there is no permissive fallback or hidden lease in this run.
5. The selected torque is bit-exact to the r137 effective command on every executable tick. Plant motion, fall-safe authority, terminal outcomes, and every fall boundary therefore remain byte-for-byte retained while the diagnostics become physically truthful about support.

## Scope

This admits the semantic current-observation composition, not a stronger recovery action or hardware controller. R179's guarded flight action remains separately conditioned; the next action experiment may replace the fixed retained torque only behind the same typed current-support proof, scheduling, robustness, and non-regression gates.
