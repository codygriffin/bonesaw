# Bonesaw support-free inexact-hold forecast selector A/B · r188

> Mechanism **PASS** · plant consequence **REJECTED** · synchronous profile **REJECTED**.

## Selector result

- Stored first-run work: **51,423 control ticks** across **63 profiles**, each with an exact replay; selector queries: **247**.
- Q15 selection counts across selector arms: **{'0': 76, '8192': 6, '16384': 5, '24576': 1, '32768': 159}**.
- Worst selector call: **6.192 µs**; Rust allocation and Python GC: **zero**.
- Timing: **100** 5 ms overruns, **6.247 ms** worst loop, and **5.866 ms** worst controller call.
- An independent same-revision full-process repetition observed **104** overruns, a **3.015 µs** selector maximum, a **6.122 ms** loop maximum, and a **5.697 ms** controller maximum. Both timing realizations reject the deadline profile; semantic outcomes and selector counts agree.
- The matched 5/10 ms pair has an identical first-loss state, score vector, selected authority, and torque; no future burst-duration input exists.

## Comparator summary

| profile | new green falls | earlier falls | worst Δ s | verdict |
|---|---|---|---|---|
| drop5_hold0 | 0 | 3 | -1.245 | REJECTED |
| drop10_hold0 | 0 | 2 | -0.155 | REJECTED |
| drop5_full | 0 | 1 | -0.040 | REJECTED |
| drop10_full | 0 | 2 | -1.655 | REJECTED |
| drop5_selector | 1 | 3 | -0.880 | REJECTED |
| drop10_selector | 0 | 3 | -1.995 | REJECTED |

## Plant consequence

| case | dropout | exact | selector | fall Δ s |
|---|---|---|---|---|
| nominal | drop5 | RECOVERED | RECOVERED | — |
| nominal | drop10 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | RECOVERED | FALL 3.000s | — |
| backward_4n | drop10 | RECOVERED | RECOVERED | — |
| left_1n | drop5 | FALL 2.470s | FALL 2.095s | -0.375 |
| left_1n | drop10 | FALL 2.470s | FALL 2.240s | -0.230 |
| right_1n_mirror | drop5 | FALL 2.520s | FALL 2.375s | -0.145 |
| right_1n_mirror | drop10 | FALL 2.520s | FALL 2.210s | -0.310 |
| handle_forward_4n | drop5 | FALL 4.630s | FALL 3.750s | -0.880 |
| handle_forward_4n | drop10 | FALL 4.630s | FALL 2.635s | -1.995 |
| forward_4n_friction_0p03 | drop5 | FALL 1.610s | FALL 1.625s | +0.015 |
| forward_4n_friction_0p03 | drop10 | FALL 1.610s | FALL 1.625s | +0.015 |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| all_arms_replay_exact | PASS |
| exact_selector_is_dormant | PASS |
| selector_queries_every_unavailable_tick | PASS |
| selector_is_unavailable_only_argmin_and_exact | PASS |
| first_loss_has_no_burst_duration_oracle | PASS |
| finite_without_numeric_fault | PASS |
| zero_rust_allocation_and_python_gc | PASS |

## Contract

- Rust forces support unknown, scores 0/0.25/0.5/0.75/1 over the fixed eight-knot reduced model, and resolves exact ties toward lower authority.
- The selected fraction scales only the last admitted effective effort; it never creates a contact-force witness or refreshes primary health.
- This is a state-local forecast witness, not a physics rollout or recovery certificate; strict plant non-regression remains the promotion gate.
