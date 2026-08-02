# Bonesaw bounded-planner exact-witness A/B · r173

> Exact host-native evaluation profile **FAIL** · controller promotion **NO**.

## Result

Both arms retain r169's 512-sweep planner prefix, 0.1 exact-continuation threshold, and uncapped final authority. The candidate may copy a terminal exact planner seed across those two stopping controls into the independent final workspace. This compatibility is one-way: the destination must be uncapped, every structural/arithmetic/bound/row bit must match, and exhausted or prefix-dependent seeds keep the complete profile key and fall back to an ordinary final solve.
Across **20** frozen cases, synchronous misses change **3 → 2**. Rust records **11,185** copies, **11,131** cross-profile exact hits, and **54** refused copies. Logical work remains **18,332,262 → 18,332,262**; complete non-timing authority/plant traces and replay are exact.
Worst final-WBC p99 changes **2.273 → 2.342 ms**; candidate maximum is **3.046 ms**. The copy boundary itself is **6.793 µs** worst-case with **0 calls / 0 bytes**.

| case | outcome | miss C | miss T | copies | hits | refused | final p99 C µs | final p99 T µs | authority exact | replay exact |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | 0 | 0 | 681 | 678 | 3 | 148 | 160 | YES | YES |
| forward_2n | FALL 3.680s | 1 | 0 | 736 | 735 | 1 | 155 | 141 | YES | YES |
| forward_4n_reference | FALL 2.865s | 1 | 1 | 573 | 568 | 5 | 563 | 168 | YES | YES |
| forward_6n_overload | FALL 1.230s | 0 | 0 | 246 | 246 | 0 | 134 | 130 | YES | YES |
| backward_2n | FALL 3.420s | 0 | 0 | 684 | 681 | 3 | 140 | 134 | YES | YES |
| backward_4n | FALL 2.730s | 0 | 0 | 546 | 544 | 2 | 143 | 140 | YES | YES |
| left_1n | FALL 2.295s | 0 | 1 | 459 | 451 | 8 | 2273 | 2342 | YES | YES |
| left_2n | FALL 2.630s | 0 | 0 | 526 | 522 | 4 | 147 | 142 | YES | YES |
| left_4n | FALL 1.850s | 0 | 0 | 370 | 369 | 1 | 143 | 145 | YES | YES |
| right_2n | FALL 2.030s | 0 | 0 | 406 | 402 | 4 | 177 | 161 | YES | YES |
| right_4n | FALL 2.195s | 0 | 0 | 439 | 435 | 4 | 151 | 152 | YES | YES |
| diagonal_4n | FALL 2.605s | 0 | 0 | 521 | 519 | 2 | 139 | 168 | YES | YES |
| up_4n | FALL 4.215s | 0 | 0 | 843 | 842 | 1 | 145 | 143 | YES | YES |
| down_4n | FALL 3.450s | 0 | 0 | 690 | 683 | 7 | 367 | 359 | YES | YES |
| handle_forward_4n | FALL 2.045s | 0 | 0 | 409 | 408 | 1 | 150 | 145 | YES | YES |
| short_8n_50ms | FALL 1.785s | 0 | 0 | 357 | 356 | 1 | 151 | 144 | YES | YES |
| long_2n_200ms | FALL 3.155s | 0 | 0 | 631 | 627 | 4 | 158 | 157 | YES | YES |
| forward_2n_three_pulses | FALL 3.625s | 1 | 0 | 725 | 722 | 3 | 150 | 150 | YES | YES |
| forward_4n_friction_0p1 | RECOVERED | 0 | 0 | 1200 | 1200 | 0 | 144 | 130 | YES | YES |
| forward_4n_friction_0p03 | FALL 0.715s | 0 | 0 | 143 | 143 | 0 | 158 | 153 | YES | YES |

## Admission gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| established_fallback_exercised | PASS |
| witness_copy_exercised | PASS |
| cross_profile_exact_hit_exercised | PASS |
| no_hit_without_copy | PASS |
| control_has_no_cross_session_hits | PASS |
| logical_projection_work_exact | PASS |
| final_projection_work_exact | PASS |
| planner_projection_work_exact | PASS |
| authority_trace_exact | PASS |
| execution_trace_exact | PASS |
| candidate_replay_exact | PASS |
| mechanism_replay_exact | PASS |
| zero_authoritative_wbc_deadline_overruns | PASS |
| zero_synchronous_loop_deadline_overruns | FAIL |
| zero_rust_allocation | PASS |
| zero_witness_transfer_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |
| no_new_numeric_fault | PASS |
| no_new_fall | PASS |

## Architecture contract

- Exact compatibility ignores only the planner projection ceiling and continuation threshold, and only toward an uncapped destination. Dimensions, ordered stable IDs, every coefficient and bound bit, singular tolerance, active-set budget, equality-repair mode, and sparse arithmetic mode still match exactly.
- A cap below the common initial Dykstra prefix is compatible only when no active-set pseudoinverse influenced the seed. Reverse uncapped-to-bounded reuse is forbidden.
- Exhausted results may reuse only under the complete identical profile, remain typed `MaxIterations`, and never enter the soft hierarchy. Refused copies execute the ordinary destination solve.
- Equality factors and every soft task are recomputed in destination-owned storage. No mutable matrix, multiplier, controller state, command, or authority token is shared.
- Python owns the frozen corpus and statistics. Rust owns the key, copy, one-way compatibility proof, revalidation, typed result, logical work, and allocation witness.

## Scope

Timing is host-descriptive for `-C target-cpu=native`. This admits an evaluation compute profile, not the experimental viability policy, a hardware controller, real-time-kernel WCET, or learned realization model. The retained r137 authority remains live.
