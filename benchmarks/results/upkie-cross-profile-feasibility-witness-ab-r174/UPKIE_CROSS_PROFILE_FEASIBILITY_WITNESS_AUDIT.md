# Bonesaw bounded-planner witness continuation A/B · r174

> Exact host-native evaluation profile **PASS** · controller promotion **NO**.

## Result

Both arms retain r169's 512-sweep planner prefix, 0.1 exact-continuation threshold, and uncapped final authority. The candidate copies either a terminal exact planner seed or its immutable exhausted Dykstra prefix into the independent final workspace. Exact seeds are revalidated directly; an exhausted bounded prefix resumes from the copied point and row multipliers through the destination's uncapped budget. Compatibility is one-way and every non-stopping-control key bit must match.
Across **20** frozen cases, synchronous misses change **5 → 0**. Rust records **11,185** copies, **11,131** terminal exact hits, and **54** bounded-prefix resumptions. Logical work remains **18,332,262 → 18,332,262**; complete non-timing authority/plant traces and replay are exact.
Worst final-WBC p99 changes **2.253 → 2.667 ms**; candidate maximum is **2.945 ms**. The copy boundary itself is **7.134 µs** worst-case with **0 calls / 0 bytes**.

| case | outcome | miss C | miss T | copies | hits | prefix | final p99 C µs | final p99 T µs | authority exact | replay exact |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | 0 | 0 | 681 | 681 | 3 | 173 | 166 | YES | YES |
| forward_2n | FALL 3.680s | 1 | 0 | 736 | 736 | 1 | 143 | 165 | YES | YES |
| forward_4n_reference | FALL 2.865s | 2 | 0 | 573 | 573 | 5 | 808 | 152 | YES | YES |
| forward_6n_overload | FALL 1.230s | 0 | 0 | 246 | 246 | 0 | 133 | 137 | YES | YES |
| backward_2n | FALL 3.420s | 0 | 0 | 684 | 684 | 3 | 149 | 154 | YES | YES |
| backward_4n | FALL 2.730s | 0 | 0 | 546 | 546 | 2 | 147 | 149 | YES | YES |
| left_1n | FALL 2.295s | 1 | 0 | 459 | 459 | 8 | 2253 | 2667 | YES | YES |
| left_2n | FALL 2.630s | 0 | 0 | 526 | 526 | 4 | 148 | 156 | YES | YES |
| left_4n | FALL 1.850s | 0 | 0 | 370 | 370 | 1 | 146 | 154 | YES | YES |
| right_2n | FALL 2.030s | 0 | 0 | 406 | 406 | 4 | 175 | 163 | YES | YES |
| right_4n | FALL 2.195s | 0 | 0 | 439 | 439 | 4 | 186 | 191 | YES | YES |
| diagonal_4n | FALL 2.605s | 0 | 0 | 521 | 521 | 2 | 150 | 148 | YES | YES |
| up_4n | FALL 4.215s | 0 | 0 | 843 | 843 | 1 | 148 | 167 | YES | YES |
| down_4n | FALL 3.450s | 0 | 0 | 690 | 690 | 7 | 368 | 308 | YES | YES |
| handle_forward_4n | FALL 2.045s | 0 | 0 | 409 | 409 | 1 | 139 | 143 | YES | YES |
| short_8n_50ms | FALL 1.785s | 0 | 0 | 357 | 357 | 1 | 141 | 146 | YES | YES |
| long_2n_200ms | FALL 3.155s | 0 | 0 | 631 | 631 | 4 | 147 | 138 | YES | YES |
| forward_2n_three_pulses | FALL 3.625s | 1 | 0 | 725 | 725 | 3 | 143 | 162 | YES | YES |
| forward_4n_friction_0p1 | RECOVERED | 0 | 0 | 1200 | 1200 | 0 | 145 | 136 | YES | YES |
| forward_4n_friction_0p03 | FALL 0.715s | 0 | 0 | 143 | 143 | 0 | 162 | 158 | YES | YES |

## Admission gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| established_fallback_exercised | PASS |
| witness_copy_exercised | PASS |
| cross_profile_witness_hit_exercised | PASS |
| no_hit_without_copy | PASS |
| every_copy_revalidated_and_consumed | PASS |
| bounded_prefix_resume_exercised | PASS |
| control_has_no_cross_session_hits | PASS |
| logical_projection_work_exact | PASS |
| final_projection_work_exact | PASS |
| planner_projection_work_exact | PASS |
| authority_trace_exact | PASS |
| execution_trace_exact | PASS |
| candidate_replay_exact | PASS |
| mechanism_replay_exact | PASS |
| zero_authoritative_wbc_deadline_overruns | PASS |
| zero_synchronous_loop_deadline_overruns | PASS |
| zero_rust_allocation | PASS |
| zero_witness_transfer_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |
| no_new_numeric_fault | PASS |
| no_new_fall | PASS |

## Architecture contract

- Exact compatibility ignores only the planner projection ceiling and continuation threshold, and only toward an uncapped destination. Dimensions, ordered stable IDs, every coefficient and bound bit, singular tolerance, active-set budget, equality-repair mode, and sparse arithmetic mode still match exactly.
- A cap below the common initial Dykstra prefix is compatible only when no active-set pseudoinverse influenced an exact seed. Exhausted prefixes require the common initial active-set attempt and preserve the exact Dykstra point plus every row multiplier. Reverse uncapped-to-bounded reuse is forbidden.
- A resumed prefix continues to the uncapped destination's ordinary terminal result and reports the complete cold-equivalent logical work. If it remains exhausted, it stays typed `MaxIterations` and never enters the soft hierarchy.
- Equality factors and every soft task are recomputed in destination-owned storage. No mutable matrix, multiplier, controller state, command, or authority token is shared.
- Python owns the frozen corpus and statistics. Rust owns the key, copy, one-way compatibility proof, revalidation, typed result, logical work, and allocation witness.

## Scope

Timing is host-descriptive for `-C target-cpu=native`. This admits an evaluation compute profile, not the experimental viability policy, a hardware controller, real-time-kernel WCET, or learned realization model. The retained r137 authority remains live.
