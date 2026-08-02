# Bonesaw equality-first feasibility plant A/B · r165

> Timing mechanism **PASS** · physical gate **FAIL** · profile/controller promotion **NO/NO**.

## Result

The candidate changes one default-off feasibility rule: when the bounded Dykstra prefix hands the active-set accelerator a point displaced from hard equalities, it solves the equality block before searching inequality violations. Exact sparse-nonzero row traversal is enabled on both arms. This may change solver/plant execution, so timing and physical gates are independent.

Logical half-space projections change **11,904,894 → 942,630** and >5 ms loops **32 → 0**. Green rows lost: **0**; earlier/later/neutral boundaries: **11/4/5**; worst/best delta **-0.485/+0.090 s**.

## Retained causal matrix

| case | control | repair | overrun C | overrun R | max sweep C | max sweep R | boundary Δs | replay |
|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | FALL 3.400s | 1 | 0 | 2496 | 8 | -0.005 | YES |
| forward_2n | FALL 3.680s | FALL 3.670s | 2 | 0 | 2688 | 8 | -0.010 | YES |
| forward_4n_reference | FALL 2.865s | FALL 2.630s | 5 | 0 | 2688 | 8 | -0.235 | YES |
| forward_6n_overload | FALL 1.230s | FALL 1.230s | 0 | 0 | 1 | 1 | +0.000 | YES |
| backward_2n | FALL 3.420s | FALL 3.490s | 2 | 0 | 2688 | 8 | +0.070 | YES |
| backward_4n | FALL 2.730s | FALL 2.690s | 2 | 0 | 2688 | 8 | -0.040 | YES |
| left_1n | FALL 2.295s | FALL 1.810s | 2 | 0 | 2688 | 8 | -0.485 | YES |
| left_2n | FALL 2.630s | FALL 2.575s | 1 | 0 | 2496 | 8 | -0.055 | YES |
| left_4n | FALL 1.850s | FALL 1.840s | 0 | 0 | 2496 | 8 | -0.010 | YES |
| right_2n | FALL 2.030s | FALL 1.845s | 1 | 0 | 2496 | 8 | -0.185 | YES |
| right_4n | FALL 2.195s | FALL 2.120s | 4 | 0 | 2056 | 8 | -0.075 | YES |
| diagonal_4n | FALL 2.605s | FALL 2.415s | 2 | 0 | 2688 | 8 | -0.190 | YES |
| up_4n | FALL 4.215s | FALL 4.210s | 0 | 0 | 2496 | 8 | -0.005 | YES |
| down_4n | FALL 3.450s | FALL 3.540s | 4 | 0 | 2688 | 8 | +0.090 | YES |
| handle_forward_4n | FALL 2.045s | FALL 2.045s | 1 | 0 | 2380 | 8 | +0.000 | YES |
| short_8n_50ms | FALL 1.785s | FALL 1.790s | 1 | 0 | 2393 | 8 | +0.005 | YES |
| long_2n_200ms | FALL 3.155s | FALL 3.155s | 1 | 0 | 2496 | 8 | +0.000 | YES |
| forward_2n_three_pulses | FALL 3.625s | FALL 3.630s | 3 | 0 | 2496 | 8 | +0.005 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | 0 | 0 | 1 | 1 | +0.000 | YES |
| forward_4n_friction_0p03 | FALL 0.715s | FALL 0.715s | 0 | 0 | 1 | 1 | +0.000 | YES |

## Gates

| gate | result |
|---|---|
| retained_matrix_complete | PASS |
| candidate_replay_exact | PASS |
| fallback_repair_exercised | PASS |
| zero_deadline_overruns | PASS |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |
| all_control_green_rows_preserved | PASS |
| no_earlier_fall_boundary | FAIL |

## Authority boundary

A timing-mechanism pass cannot admit this profile if the physical gate fails. A changed feasible seed is not an arithmetic-only optimization; it remains default-off until both exact solver semantics and downstream consequences are accepted.
