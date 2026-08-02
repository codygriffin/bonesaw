# Bonesaw signed-zero-preserving sparse feasibility A/B · r170

> Exact CPU profile **FAIL** · controller promotion **NO**.

## Result

The candidate traverses compiled nonzero coordinate/value lists for Dykstra dot and transpose work, but explicitly reproduces the dense kernel's `-0 → +0` transitions for omitted signed-zero coefficients. This preserves bitwise active-set tie/freeze inputs without executing every zero multiply. Both planner and executable WBC use the same kernel; neither has a projection cap or equality-first repair.
Across **20** frozen cases, >5 ms loops change **60 → 32**, logical projections remain **11,904,894 → 11,904,894**, and the complete non-timing trace is **exact**.

## Retained causal matrix

| case | dense | sparse signed-zero | overrun D | overrun S | final WBC p99 D µs | final WBC p99 S µs | complete exact | replay exact |
|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | FALL 3.405s | 4 | 1 | 151 | 179 | YES | YES |
| forward_2n | FALL 3.680s | FALL 3.680s | 2 | 2 | 154 | 158 | YES | YES |
| forward_4n_reference | FALL 2.865s | FALL 2.865s | 6 | 5 | 1365 | 574 | YES | YES |
| forward_6n_overload | FALL 1.230s | FALL 1.230s | 0 | 0 | 148 | 150 | YES | YES |
| backward_2n | FALL 3.420s | FALL 3.420s | 3 | 2 | 153 | 148 | YES | YES |
| backward_4n | FALL 2.730s | FALL 2.730s | 2 | 2 | 150 | 202 | YES | YES |
| left_1n | FALL 2.295s | FALL 2.295s | 8 | 4 | 6040 | 2248 | YES | YES |
| left_2n | FALL 2.630s | FALL 2.630s | 4 | 1 | 152 | 150 | YES | YES |
| left_4n | FALL 1.850s | FALL 1.850s | 1 | 0 | 150 | 145 | YES | YES |
| right_2n | FALL 2.030s | FALL 2.030s | 4 | 0 | 170 | 184 | YES | YES |
| right_4n | FALL 2.195s | FALL 2.195s | 4 | 4 | 181 | 164 | YES | YES |
| diagonal_4n | FALL 2.605s | FALL 2.605s | 2 | 2 | 156 | 155 | YES | YES |
| up_4n | FALL 4.215s | FALL 4.215s | 3 | 0 | 242 | 162 | YES | YES |
| down_4n | FALL 3.450s | FALL 3.450s | 7 | 3 | 751 | 404 | YES | YES |
| handle_forward_4n | FALL 2.045s | FALL 2.045s | 1 | 1 | 145 | 158 | YES | YES |
| short_8n_50ms | FALL 1.785s | FALL 1.785s | 1 | 1 | 184 | 179 | YES | YES |
| long_2n_200ms | FALL 3.155s | FALL 3.155s | 4 | 0 | 166 | 162 | YES | YES |
| forward_2n_three_pulses | FALL 3.625s | FALL 3.625s | 4 | 4 | 167 | 171 | YES | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | 0 | 0 | 145 | 147 | YES | YES |
| forward_4n_friction_0p03 | FALL 0.715s | FALL 0.715s | 0 | 0 | 164 | 165 | YES | YES |

## Gates

| gate | result |
|---|---|
| retained_matrix_complete | PASS |
| complete_trace_exact | PASS |
| projection_work_exact | PASS |
| candidate_replay_exact | PASS |
| zero_deadline_overruns | FAIL |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |

## Scope

This may admit an exact default-off CPU kernel profile only. Host timing is unisolated; controller promotion remains false and the retained live r137 authority is unchanged.
