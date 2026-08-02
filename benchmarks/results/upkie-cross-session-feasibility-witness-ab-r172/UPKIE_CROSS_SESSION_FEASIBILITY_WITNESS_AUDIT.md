# Bonesaw copied hard-feasibility witness A/B · r172

> Copied-witness mechanism **FAIL** · r169-profile semantic promotion **FAIL** · controller promotion **NO**.

## Result

The established arm retains r169's exact 512-sweep planner prefix. The matched diagnostic control removes that prefix, while the transfer candidate preserves it and copies only budget-independent feasible terminals into the uncapped final-authority workspace. Rust rebuilds and bit-compares every structural/arithmetic hard-problem bit; projection ceiling and continuation slots may differ only for a feasible terminal proven independent of those stopping controls. Exhausted or prefix-dependent witnesses miss the cache and force an ordinary cold final solve. No mutable matrices, multipliers, controller state, or session storage are shared, and the final session recomputes every soft hierarchy.
Across **20** frozen plant cases, synchronous misses are **3 established / 34 matched-control → 1 transfer**. Rust records **11,185** immutable transfer attempts and **11,131** revalidated portable final hits; every other attempt falls back cold. Logical half-space work remains **18,332,262 → 18,332,262**, the established complete non-timing authority/plant trace is **exact**, and replay is exact.
Aligning the planner profile with final authority preserves the established executable-request trace in **20/20** cases and plant execution in **20/20**; full authority telemetry is exact in **10/20**, with any difference confined to non-executable speculative diagnostics.
Worst final-WBC p99 changes **2.278 → 2.294 ms**; candidate maximum is **3.148 ms**.
The copied-witness boundary itself is explicitly timed and allocation-counted: worst **1.142 µs**, **0 calls / 0 bytes** across the matrix.

| case | established | control | transfer | miss E | miss C | miss T | copies | hits | final p99 E µs | final p99 T µs | E/C exec exact | authority exact | replay exact |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | FALL 3.405s | FALL 3.405s | 0 | 1 | 0 | 681 | 678 | 141 | 151 | YES | YES | YES |
| forward_2n | FALL 3.680s | FALL 3.680s | FALL 3.680s | 1 | 2 | 0 | 736 | 735 | 211 | 165 | YES | YES | YES |
| forward_4n_reference | FALL 2.865s | FALL 2.865s | FALL 2.865s | 1 | 5 | 1 | 573 | 568 | 566 | 169 | YES | YES | YES |
| forward_6n_overload | FALL 1.230s | FALL 1.230s | FALL 1.230s | 0 | 0 | 0 | 246 | 246 | 136 | 148 | YES | YES | YES |
| backward_2n | FALL 3.420s | FALL 3.420s | FALL 3.420s | 0 | 2 | 0 | 684 | 681 | 171 | 150 | YES | YES | YES |
| backward_4n | FALL 2.730s | FALL 2.730s | FALL 2.730s | 0 | 2 | 0 | 546 | 544 | 147 | 148 | YES | YES | YES |
| left_1n | FALL 2.295s | FALL 2.295s | FALL 2.295s | 0 | 5 | 0 | 459 | 451 | 2278 | 2294 | YES | YES | YES |
| left_2n | FALL 2.630s | FALL 2.630s | FALL 2.630s | 0 | 2 | 0 | 526 | 522 | 147 | 148 | YES | YES | YES |
| left_4n | FALL 1.850s | FALL 1.850s | FALL 1.850s | 0 | 0 | 0 | 370 | 369 | 144 | 140 | YES | YES | YES |
| right_2n | FALL 2.030s | FALL 2.030s | FALL 2.030s | 0 | 4 | 0 | 406 | 402 | 162 | 163 | YES | YES | YES |
| right_4n | FALL 2.195s | FALL 2.195s | FALL 2.195s | 0 | 2 | 0 | 439 | 435 | 153 | 157 | YES | YES | YES |
| diagonal_4n | FALL 2.605s | FALL 2.605s | FALL 2.605s | 0 | 2 | 0 | 521 | 519 | 150 | 150 | YES | YES | YES |
| up_4n | FALL 4.215s | FALL 4.215s | FALL 4.215s | 0 | 0 | 0 | 843 | 842 | 145 | 141 | YES | YES | YES |
| down_4n | FALL 3.450s | FALL 3.450s | FALL 3.450s | 0 | 2 | 0 | 690 | 683 | 510 | 374 | YES | YES | YES |
| handle_forward_4n | FALL 2.045s | FALL 2.045s | FALL 2.045s | 0 | 1 | 0 | 409 | 408 | 147 | 136 | YES | YES | YES |
| short_8n_50ms | FALL 1.785s | FALL 1.785s | FALL 1.785s | 0 | 1 | 0 | 357 | 356 | 155 | 140 | YES | YES | YES |
| long_2n_200ms | FALL 3.155s | FALL 3.155s | FALL 3.155s | 0 | 1 | 0 | 631 | 627 | 178 | 176 | YES | YES | YES |
| forward_2n_three_pulses | FALL 3.625s | FALL 3.625s | FALL 3.625s | 1 | 2 | 0 | 725 | 722 | 149 | 172 | YES | YES | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 0 | 0 | 0 | 1200 | 1200 | 165 | 136 | YES | YES | YES |
| forward_4n_friction_0p03 | FALL 0.715s | FALL 0.715s | FALL 0.715s | 0 | 0 | 0 | 143 | 143 | 151 | 154 | YES | YES | YES |

## Admission gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| established_fallback_exercised | PASS |
| witness_transfer_exercised | PASS |
| portable_witness_hits_are_bounded_by_transfers | PASS |
| control_has_no_cross_session_hits | PASS |
| established_execution_trace_exact | PASS |
| established_executable_request_exact | PASS |
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

- The witness is a copied terminal hard seed plus exact problem key and logical diagnostics. It is not a shared workspace, warm soft solution, command, or authority token.
- The destination assembles and bit-compares its own complete ordered hard problem. Any coefficient, bound, stable ID, feasibility profile, or dimension mismatch forces an ordinary cold solve.
- Equality factors are built in destination-owned storage. Every soft task is independently solved after hard feasibility reuse.
- Exhausted results remain typed `MaxIterations`; they cannot acquire torque authority through transfer.
- Python owns frozen plant sequencing and report generation only. Rust owns the key, copy, revalidation, solve, typed status, logical counters, and allocation witness.

## Scope

Timing is descriptive for this host and explicit `-C target-cpu=native` build. This is a policy-free CPU WBC/profile admission test, not a hardware controller, real-time-kernel WCET proof, or learned realization certificate. The retained r137 authority stays live.
