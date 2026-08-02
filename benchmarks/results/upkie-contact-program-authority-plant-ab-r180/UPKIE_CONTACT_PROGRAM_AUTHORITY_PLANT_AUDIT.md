# Bonesaw contact-program authority plant A/B · r180

> Mechanism **PASS** · authoritative CPU WBC profile **REJECTED** · synchronous supervisor profile **REJECTED** · hardware controller promotion **NO**.

## Result

The Rust authority machine now distinguishes a stable primary program, a separately admitted current-hard-support realization, a bounded retained lease, and withheld authority. Startup cannot jump directly to a reduced/flight command: three explicit prestart contact observations establish causal contact evidence, and the first executable program must be primary.

The current-support WBC does not invent a new torque. It fixes the active primary-program actuator effort, re-solves floating dynamics/contact against the observed hard mask, and returns an actual-support acceleration/force witness. During debounce, this candidate may become fresh only after prior authority exists. Evidence loss, inconsistent masks, invalid sequence, rejection, and lease expiry remain fail-closed.

This evaluated profile configures the retained-command lease to **zero ticks**. A two-tick stale hold changed red-row plant boundaries and was rejected. Contact transition is covered by continuously fresh, separately admitted current-support realization; if that query is unavailable, authority is withheld rather than extended by an old command.

| case | r137 sentinel | r180 candidate | fall Δ s | fresh current | during transition | dense→sparse overruns | loop p99 µs |
|---|---|---|---|---|---|---|---|
| backward_2n | RECOVERED | RECOVERED | — | 1 | 1 | 0→0 | 552 |
| backward_4n | RECOVERED | RECOVERED | — | 4 | 4 | 0→0 | 509 |
| diagonal_4n | FALL 2.125s | FALL 2.125s | +0.000 | 112 | 47 | 63→0 | 3941 |
| down_4n | RECOVERED | RECOVERED | — | 0 | 0 | 0→0 | 495 |
| forward_2n | RECOVERED | RECOVERED | — | 1 | 1 | 0→0 | 528 |
| forward_2n_three_pulses | RECOVERED | RECOVERED | — | 2 | 2 | 0→0 | 549 |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 1.610s | +0.000 | 71 | 14 | 36→1 | 4457 |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | — | 6 | 6 | 0→0 | 560 |
| forward_4n_reference | RECOVERED | RECOVERED | — | 3 | 3 | 0→0 | 529 |
| forward_6n_overload | FALL 4.820s | FALL 4.820s | +0.000 | 115 | 43 | 74→0 | 4090 |
| handle_forward_4n | FALL 4.630s | FALL 4.630s | +0.000 | 201 | 60 | 95→1 | 4145 |
| left_1n | FALL 2.470s | FALL 2.470s | +0.000 | 138 | 39 | 69→3 | 4392 |
| left_2n | FALL 2.350s | FALL 2.350s | +0.000 | 139 | 36 | 84→1 | 4616 |
| left_4n | FALL 2.610s | FALL 2.610s | +0.000 | 299 | 80 | 138→2 | 4737 |
| long_2n_200ms | RECOVERED | RECOVERED | — | 5 | 5 | 0→0 | 577 |
| nominal | RECOVERED | RECOVERED | — | 0 | 0 | 0→0 | 533 |
| right_2n | FALL 2.015s | FALL 2.015s | +0.000 | 28 | 12 | 15→0 | 3229 |
| right_4n | FALL 2.415s | FALL 2.415s | +0.000 | 96 | 22 | 72→1 | 4462 |
| short_8n_50ms | RECOVERED | RECOVERED | — | 6 | 6 | 0→0 | 560 |
| up_4n | RECOVERED | RECOVERED | — | 0 | 0 | 0→0 | 560 |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| preservation_shadow_matches_r137_physics | PASS |
| sparse_row_kernel_is_semantically_exact | PASS |
| candidate_replay_exact | PASS |
| candidate_matches_r137_execution | PASS |
| startup_establishes_primary_authority_first | PASS |
| current_support_authority_exercised | PASS |
| transition_current_support_authority_exercised | PASS |
| execution_only_with_consistent_masks | PASS |
| typed_fresh_and_retained_provenance | PASS |
| current_support_is_separately_admitted_and_requested | PASS |
| current_support_uses_actual_reduced_hard_mask | PASS |
| fixed_effort_realization_preserves_primary_program_torque | PASS |
| bounded_lease_and_zero_when_withheld | PASS |
| no_realization_fallback | PASS |
| candidate_finite_and_allocation_free | PASS |
| zero_python_gc | PASS |

## Promotion gates

| gate | result |
|---|---|
| retained_green_rows_preserved | PASS |
| no_r137_fall_boundary_earlier | PASS |
| no_candidate_numeric_fault | PASS |
| zero_5ms_loop_overruns | FAIL |
| candidate_wbc_calls_below_5ms | FAIL |
| delay_noise_dropout_and_mirrored_evidence_complete | FAIL |
| hardware_or_higher_fidelity_plant_evidence_complete | FAIL |

## Timing and memory

- Dense feasibility rows miss the 5 ms synchronous deadline **646** times; the established sparse-row kernel removes those misses without changing any semantic trace: **9** remain.
- The current-support realization is selected for **1227** ticks, including **381** debounce ticks. The configured stale-command hold is **0 ticks** and observed retained-lease execution is **0 ticks**.
- Every reported candidate run records zero Rust hot-path allocation and zero Python GC collection. Per-row RSS deltas are retained in the JSON but are descriptive process-residency measurements, not a deterministic ownership bound.
- Timing is host-native descriptive evidence on an ordinary Linux process, not a real-time scheduling guarantee. No core isolation, priority elevation, or real-time kernel is claimed.

## Consequence

- Retained r137 green rows lost: **0** (none).
- Earlier r137 fall boundaries: **0** (none).
- Later fall boundaries: **0** (none).
- The authoritative WBC profile can be admitted independently of ordinary-process scheduling jitter only when every measured controller call remains inside 5 ms. The combined synchronous supervisor still requires zero measured loop misses. Neither result admits hardware authority: delay/noise/dropout, mirrored evidence, estimator disagreement, coupled actuation, thermal/power envelopes, and a higher-fidelity or hardware plant remain open.

## Authority stack

1. Exact timestamped contact evidence and debounce state determine raw, stable, and hard masks.
2. Stable double support may admit the established primary WBC command.
3. Reduced hard support may admit a fixed-primary-program-effort realization through an independent WBC query.
4. Rust contact-program authority selects fresh primary, fresh current support, bounded retained lease, or withheld; this profile sets the retained lease budget to zero.
5. The fall-safe/execution boundary emits only the selected fixed-size command; unavailable authority emits zero torque.
