# Bonesaw causal support-contingency plant A/B · r176

> Mechanism **PASS** · controller promotion **REJECTED**. Four authority arms retain r137 sentinel, measured-contact control, non-executing R175 shadow, and causal R175 selection; the selected arm is rerun for exact replay.

## Consequence

| case | r137 | measured control | selected candidate | selected ticks | fall Δ s |
|---|---|---|---|---|---|
| backward_2n | RECOVERED | FALL 3.420s | FALL 4.080s | 118 | +0.660 |
| backward_4n | RECOVERED | FALL 2.730s | FALL 2.695s | 59 | -0.035 |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.730s | 79 | +0.125 |
| forward_2n | RECOVERED | FALL 3.680s | FALL 3.805s | 118 | +0.125 |
| forward_4n_reference | RECOVERED | FALL 2.865s | FALL 2.210s | 57 | -0.655 |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 1.300s | 31 | +0.070 |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 2.325s | 144 | +0.030 |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 2.265s | 60 | -0.270 |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 1.740s | 47 | -0.110 |
| nominal | RECOVERED | FALL 3.395s | FALL 4.260s | 204 | +0.865 |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 1.945s | 53 | -0.085 |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 1.920s | 50 | -0.275 |
| up_4n | RECOVERED | FALL 4.215s | FALL 3.675s | 59 | -0.540 |
| down_4n | RECOVERED | FALL 3.450s | FALL 3.820s | 142 | +0.370 |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 2.430s | 150 | +0.385 |
| short_8n_50ms | RECOVERED | FALL 1.785s | FALL 2.370s | 107 | +0.585 |
| long_2n_200ms | RECOVERED | FALL 3.155s | RECOVERED | 5 | — |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | FALL 3.630s | 81 | +0.005 |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | FALL 2.155s | 77 | — |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 0.375s | 62 | -0.340 |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| shadow_execution_is_exactly_neutral | PASS |
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
| retained_green_rows_preserved | FAIL |
| measured_control_green_rows_preserved | FAIL |
| no_existing_fall_boundary_earlier | FAIL |
| no_candidate_numeric_fault | PASS |
| zero_5ms_loop_overruns | FAIL |

## Authority contract

- The shadow arm runs Rust FK/request authoring and a second floating WBC every tick but cannot affect torque; physical equality with measured-contact control is an explicit gate.
- Selection is causal and state-minimal: one exact `11` observation arms the run, exact current hard support later leaves `11`, the independent candidate status is `Solved`/`SolvedWithSlack`, and hard violation is below `1e-8`. No elapsed-time dwell, blend, timeout extension, cache resurrection, plant reset, or rejected-row execution participates.
- A mechanism pass proves only that selection is typed and reproducible. Promotion additionally requires every retained green row, every measured-control green row, no earlier existing fall boundary, no numeric fault, and zero 5 ms loop overruns.
