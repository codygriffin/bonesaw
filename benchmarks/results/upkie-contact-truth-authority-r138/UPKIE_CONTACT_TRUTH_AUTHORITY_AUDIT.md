# Bonesaw Upkie measured-contact authority audit · r138

> Evaluation **PASS**. Frozen-state queries contain no policy, integration, or physics step; the source snapshots come from the retained MuJoCo failure boundary.

## Outcome

The baseline plant controller declares both RollingWheel contacts hard-active on every tick, but the actual plant reaches double-, single-, and zero-wheel support before the four frozen falls. Replaying the same root/joint states and the same 250 ms roll/lateral rate-arrest request through persistent Rust changes the local authority result when measured contact activity replaces the stale declaration. `Projection gain` is the signed achieved/request projection: negative means response away from the request and values above one mean over-response, not extra health. This identifies contact observation/admission as a missing controller input; it does not authorize a transition controller or claim recovery.

| case | before fall s | measured L/R | declared projection gain | measured projection gain | declared status | measured status |
|---|---|---|---|---|---|---|
| left_1n | 0.50 | 10 | -0.376 | 46.075 | SolvedWithSlack | SolvedWithSlack |
| left_1n | 0.25 | 00 | NON-EXEC | -3.759 | MaxIterations | SolvedWithSlack |
| left_1n | 0.10 | 00 | NON-EXEC | 1.245 | MaxIterations | SolvedWithSlack |
| left_1n | 0.05 | 00 | NON-EXEC | -1.835 | MaxIterations | SolvedWithSlack |
| left_2n | 0.50 | 11 | 1.000 | 1.000 | Solved | Solved |
| left_2n | 0.25 | 10 | NON-EXEC | -8.771 | MaxIterations | SolvedWithSlack |
| left_2n | 0.10 | 00 | NON-EXEC | 0.724 | MaxIterations | SolvedWithSlack |
| left_2n | 0.05 | 00 | NON-EXEC | -1.676 | MaxIterations | SolvedWithSlack |
| right_2n | 0.50 | 11 | 1.003 | 1.003 | Solved | Solved |
| right_2n | 0.25 | 11 | 1.937 | 1.937 | Solved | Solved |
| right_2n | 0.10 | 00 | NON-EXEC | -7.283 | MaxIterations | SolvedWithSlack |
| right_2n | 0.05 | 00 | NON-EXEC | -1.850 | MaxIterations | SolvedWithSlack |
| forward_6n_overload | 0.50 | 11 | 0.999 | 0.999 | Solved | Solved |
| forward_6n_overload | 0.25 | 10 | 1.006 | 94.966 | Solved | SolvedWithSlack |
| forward_6n_overload | 0.10 | 00 | NON-EXEC | 1.721 | MaxIterations | SolvedWithSlack |
| forward_6n_overload | 0.05 | 00 | NON-EXEC | 0.209 | MaxIterations | SolvedWithSlack |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| double_single_zero_contact_discriminate | PASS |
| declared_contact_mismatch_observed | PASS |
| authority_result_changes | PASS |
| measured_contact_hard_rows_feasible | PASS |
| stale_double_contact_hard_failure_observed | PASS |
| exact_replay | PASS |
| zero_rust_allocation | PASS |
| finite | PASS |

## Contract

- `11`, `10`, and `00` are exact MuJoCo wheel-subtree-to-world contact observations, not inferred from total contact count. The frozen rows do not contain the mirrored `01` single-contact state, and the audit does not manufacture it.
- Declared and measured queries share identical frozen q/v/root state, request, contact basis, model, effort bounds, and solver hierarchy; only the active contact mask changes.
- The 250 ms rate-arrest vector is a diagnostic request and is never executed. The audit adds no policy or rollout.
- A future live transition needs debounced/causal contact observations and an independently admitted one-/zero-contact action; simulator truth is not a hardware estimator.
